#include "hdmi_cec.h"
#include "esphome/core/log.h"

#ifdef USE_CEC_DECODER
#include "cec_decoder.h"
#endif

namespace esphome {
namespace hdmi_cec {

static const char *const TAG = "hdmi_cec";

// ======== Receiver timing constants (in microseconds, per HDMI-CEC spec) =========
// A "start bit" is a very long low pulse that signals the beginning of a frame.
static const uint32_t START_BIT_MIN_US = 3500;

// A logic "1" bit is transmitted by pulling LOW for ~600 µs, but when *receiving*
// we don't know exact timing, only ranges. HIGH_BIT_MIN_US/HIGH_BIT_MAX_US define
// how long the LOW pulse must be to be interpreted as a "1".
static const uint32_t HIGH_BIT_MIN_US = 400;
static const uint32_t HIGH_BIT_MAX_US = 800;

// ======== Transmitter timing constants (per HDMI-CEC specification) ==============
// Each CEC bit is exactly 2.4 ms long.
static const uint32_t TOTAL_BIT_US = 2400;

// Duration of the LOW part of a "logic 1" bit
static const uint32_t HIGH_BIT_US = 600;

// Duration of the LOW part of a "logic 0" bit
static const uint32_t LOW_BIT_US = 1500;

// ======== Retransmission limits ===================================================
static const size_t MAX_ATTEMPTS = 5;

// ======== Pin configuration flags =================================================
// CEC is an open-drain, single-wire bus with required pull-up.
static const gpio::Flags INPUT_MODE_FLAGS = gpio::FLAG_INPUT | gpio::FLAG_PULLUP;
static const gpio::Flags OUTPUT_MODE_FLAGS = gpio::FLAG_OUTPUT | gpio::FLAG_OPEN_DRAIN;

// NOTE for ESP8266:
// It does NOT support enabling pull-ups when in OPEN_DRAIN output mode.
// Because of this hardware limitation, the pin is switched between:
//
//   • OUTPUT (low only)
//   • INPUT with PULLUP (for logic high)
//
// This avoids short circuits and enables arbitration checks.

// =================================================================================
// Frame constructor — builds a CEC frame including header + payload
// =================================================================================
Frame::Frame(uint8_t initiator_addr, uint8_t target_addr, const std::vector<uint8_t> &payload)
    : std::vector<uint8_t>(1 + payload.size(), (uint8_t) (0)) {

  // Header byte: high nibble = source address, low nibble = destination address
  this->at(0) = ((initiator_addr & 0xf) << 4) | (target_addr & 0xf);

  // Copy payload bytes after header
  std::memcpy(this->data() + 1, payload.data(), payload.size());
}

// Convert frame to hex representation, e.g. "04:8F:00"
// If decoder is enabled, also append readable meaning.
std::string Frame::to_string(bool skip_decode) const {
  std::string result;
  char part_buffer[3];

  // Format each byte into hex and join with ":"
  for (auto it = this->cbegin(); it != this->cend(); it++) {
    uint8_t byte_value = *it;
    sprintf(part_buffer, "%02X", byte_value);
    result += part_buffer;
    if (it != (this->end() - 1))
      result += ":";
  }

#ifdef USE_CEC_DECODER
  // Append human-readable interpretation
  if (!skip_decode) {
    Decoder decoder(*this);
    result += " => " + decoder.decode();
  }
#endif

  return result;
}

// =================================================================================
// GPIO helper functions – optimized for interrupt-level timing
// =================================================================================

// Switch pin to input mode (logic high via pull-up)
inline void IRAM_ATTR HDMICEC::set_pin_input_high() {
  pin_->pin_mode(INPUT_MODE_FLAGS);
}

// Drive pin low using open-drain output mode
inline void IRAM_ATTR HDMICEC::set_pin_output_low() {
  pin_->pin_mode(OUTPUT_MODE_FLAGS);
  pin_->digital_write(false);
}

// =================================================================================
// Driver initialization
// =================================================================================
void HDMICEC::setup() {
  this->pin_->setup();
  isr_pin_ = pin_->to_isr();             // ISR-safe pin instance
  frames_queue_.reset();                 // reset circular frame buffer
  pin_->attach_interrupt(HDMICEC::gpio_intr_, this, gpio::INTERRUPT_ANY_EDGE);
  set_pin_input_high();                  // release line (logic high)
}

void HDMICEC::dump_config() {
  ESP_LOGCONFIG(TAG, "HDMI-CEC");
  LOG_PIN("  pin: ", pin_);
  ESP_LOGCONFIG(TAG, "  address: %x", address_);
  ESP_LOGCONFIG(TAG, "  promiscuous mode: %s", (promiscuous_mode_ ? "yes" : "no"));
  ESP_LOGCONFIG(TAG, "  monitor mode: %s", (monitor_mode_ ? "yes" : "no"));
}

// =================================================================================
// Main processing loop – handles received frames and dispatches them
// =================================================================================
void HDMICEC::loop() {
  while (const Frame *frame = frames_queue_.front()) {

    // Extract addressing information
    uint8_t header = frame->front();
    uint8_t src_addr = ((header & 0xF0) >> 4);
    uint8_t dest_addr = (header & 0x0F);

    // Skip frames not addressed to us (unless broadcast or promiscuous)
    if (!promiscuous_mode_ && (dest_addr != 0x0F) && (dest_addr != address_)) {
      frames_queue_.push_front();
      continue;
    }

    // Ignore pings (1-byte frames)
    if (frame->size() == 1) {
      ESP_LOGV(TAG, "ping received: 0x%01X -> 0x%01X", src_addr, dest_addr);
      frames_queue_.push_front();
      continue;
    }

    ESP_LOGD(TAG, "[received] %s", frame->to_string().c_str());

    // Extract payload (everything except header)
    std::vector<uint8_t> data(frame->begin() + 1, frame->end());

    frames_queue_.push_front();  // free buffer for next message

    // Run trigger callbacks (automation)
    bool handled_by_trigger = false;
    uint8_t opcode = data[0];

    for (auto trigger : message_triggers_) {
      bool can_trigger =
        (!trigger->source_.has_value()      || (trigger->source_ == src_addr)) &&
        (!trigger->destination_.has_value() || (trigger->destination_ == dest_addr)) &&
        (!trigger->opcode_.has_value()      || (trigger->opcode_ == opcode)) &&
        (!trigger->data_.has_value() ||
         (data.size() == trigger->data_->size() &&
          std::equal(trigger->data_->begin(), trigger->data_->end(), data.begin()))
        );

      if (can_trigger) {
        trigger->trigger(src_addr, dest_addr, data);
        handled_by_trigger = true;
      }
    }

    // Handle default CEC messages (Get CEC Version, Power Status, OSD Name, etc.)
    bool is_direct = (dest_addr != 0xF && dest_addr == address_);
    if (is_direct && !handled_by_trigger) {
      try_builtin_handler_(src_addr, dest_addr, data);
    }
  }
}

// =================================================================================
// Map logical address to HDMI device type (used for Report Physical Address)
// =================================================================================
uint8_t logical_address_to_device_type(uint8_t logical_address) {
  switch (logical_address) {
    case 0x0: return 0x00; // TV
    case 0x5: return 0x05; // Audio System

    case 0x1: case 0x2: case 0x9:
      return 0x01; // Recording Device

    case 0x3: case 0x6: case 0x7: case 0xA:
      return 0x03; // Tuner

    default:
      return 0x04; // Playback Device
  }
}

// =================================================================================
// Built-in opcode handler – responds to common CEC requests
// =================================================================================
void HDMICEC::try_builtin_handler_(uint8_t source, uint8_t destination,
                                   const std::vector<uint8_t> &data) {
  if (data.empty()) return;

  uint8_t opcode = data[0];

  switch (opcode) {

    // === "Get CEC Version" ===
    case 0x9F:
      send(address_, source, {0x9E, 0x04}); // 0x04 = CEC version 1.4
      break;

    // === "Give Device Power Status" ===
    case 0x8F:
      send(address_, source, {0x90, 0x00}); // 0x00 = On
      break;

    // === "Give OSD Name" ===
    case 0x46: {
      std::vector<uint8_t> data = { 0x47 }; // Set OSD Name opcode
      data.insert(data.end(), osd_name_bytes_.begin(), osd_name_bytes_.end());
      send(address_, source, data);
      break;
    }

    // === "Give Physical Address" ===
    case 0x83: {
      auto phys = decode_value(physical_address_);
      std::vector<uint8_t> data = { 0x84 };  // Report Physical Address
      data.insert(data.end(), phys.begin(), phys.end());
      data.push_back(logical_address_to_device_type(address_));
      send(address_, 0xF, data); // broadcast
      break;
    }

    // Ignore Feature Abort messages
    case 0x00:
      break;

    // === Unhandled opcode => respond "Feature Abort" ===
    default:
      send(address_, source, {0x00, opcode, 0x00});
      break;
  }
}

// =================================================================================
// Send a CEC frame with retries and arbitration handling
// =================================================================================
bool HDMICEC::send(uint8_t source, uint8_t destination,
                   const std::vector<uint8_t> &data_bytes) {

  if (monitor_mode_) return false;

  bool is_broadcast = (destination == 0xF);

  Frame frame(source, destination, data_bytes);
  ESP_LOGD(TAG, "[sending] %s", frame.to_string().c_str());

  {
    LockGuard send_lock(send_mutex_);

    // Required bus idle time depends on arbitration rules
    uint8_t free_bit_periods =
      (last_sent_us_ > last_falling_edge_us_) ? 7 : 5;

    for (size_t i = 0; i < MAX_ATTEMPTS; i++) {

      // Wait until the bus is idle for the required time
      int32_t delay = 0;
      while ((delay = free_bit_periods * TOTAL_BIT_US +
                      std::max(last_sent_us_, last_falling_edge_us_) -
                      micros()) > 0) {
        ESP_LOGV(TAG, "HDMICEC::send(): waiting %d usec for bus free period", delay);
        delay_microseconds_safe(delay);
        free_bit_periods = 5; // after the first wait, always use 5 bit periods
      }

      // Attempt to send frame
      auto result = send_frame_(frame, is_broadcast);
      if (result == SendResult::Success) {
        ESP_LOGD(TAG, "frame sent and acknowledged");
        return true;
      }

      ESP_LOGI(TAG, "HDMICEC::send(): frame not sent: %s",
               (result == SendResult::BusCollision ?
                "Bus Collision" : "No Ack received"));

      // Retry using minimum wait period
      free_bit_periods = 3;
    }
  }

  ESP_LOGE(TAG, "HDMICEC::send(): send failed after five attempts");
  return false;
}

// =================================================================================
// Internal function to transmit a frame bit-by-bit
// =================================================================================
SendResult IRAM_ATTR HDMICEC::send_frame_(const Frame &frame, bool is_broadcast) {
  pin_->detach_interrupt(); // disable interrupts during transmit
  auto result = SendResult::Success;

  bool success = send_start_bit_();

  // Send each data byte in sequence
  for (auto it = frame.begin(); it != frame.end(); ++it) {
    uint8_t current_byte = *it;

    // Send byte bits MSB first
    for (int8_t i = 7; (i >= 0) && success; i--) {
      bool bit_value = ((current_byte >> i) & 0b1);

      // Arbitration only applies to initiator's address bits (first header nibble)
      if ((it == frame.begin()) && i >= 4 && bit_value) {
        success = send_high_and_test_();
      } else {
        send_bit_(bit_value);
      }
    }

    if (!success) {
      result = SendResult::BusCollision;
      break;
    }

    // Send EOM (end of message) bit
    bool is_eom = (it == (frame.end() - 1));
    send_bit_(is_eom);

    // Send ACK bit (receivers must pull line low if acknowledging)
    bool value = send_high_and_test_();
    success = (value == is_broadcast);  // Broadcast = no ACK expected
    if (!success) {
      result = SendResult::NoAck;
      break;
    }
  }

  last_sent_us_ = micros();

  pin_->attach_interrupt(HDMICEC::gpio_intr_, this, gpio::INTERRUPT_ANY_EDGE);
  return result;
}

// =================================================================================
// Send start bit (long low pulse with timing collision detection)
// =================================================================================
bool IRAM_ATTR HDMICEC::send_start_bit_() {
  // 1. Pull low for ~3.7ms
  set_pin_output_low();
  delay_microseconds_safe(3700);

  // 2. Release line (logic high)
  set_pin_input_high();
  delay_microseconds_safe(400);

  // Arbitration checks (rising edge must remain high)
  bool value = pin_->digital_read();
  delay_microseconds_safe(400);
  value &= pin_->digital_read();

  // True if no collision detected
  return (value == true);
}

// =================================================================================
// Send a single CEC bit with proper timing
// =================================================================================
void IRAM_ATTR HDMICEC::send_bit_(bool bit_value) {
  const uint32_t low_duration_us  = (bit_value ? HIGH_BIT_US : LOW_BIT_US);
  const uint32_t high_duration_us = (TOTAL_BIT_US - low_duration_us);

  set_pin_output_low();
  delay_microseconds_safe(low_duration_us);
  set_pin_input_high();
  delay_microseconds_safe(high_duration_us);
}

// =================================================================================
// Send a "high bit" and sample mid-period to check if another device overrides it
// (used for arbitration and ACK detection)
// =================================================================================
bool IRAM_ATTR HDMICEC::send_high_and_test_() {
  uint32_t start_us = micros();

  // Drive LOW briefly (600 us)
  set_pin_output_low();
  delay_microseconds_safe(HIGH_BIT_US);
  set_pin_input_high();

  static const uint32_t SAFE_SAMPLE_US = 1050;

  // Wait until safe sample point
  delay_microseconds_safe(SAFE_SAMPLE_US - (micros() - start_us));
  bool value = pin_->digital_read();

  // Finish remaining bit duration
  delay_microseconds_safe(TOTAL_BIT_US - (micros() - start_us));

  return value;
}

// =================================================================================
// GPIO interrupt handler — decodes incoming frames bit-by-bit
// =================================================================================
void IRAM_ATTR HDMICEC::gpio_intr_(HDMICEC *self) {
  const uint32_t now = micros();
  const bool level = self->isr_pin_.digital_read();

  // Ignore spurious interrupts caused by mode switching
  if (level == self->last_level_)
    return;

  self->last_level_ = level;

  // Falling edge = start of LOW pulse
  if (level == false) {
    self->last_falling_edge_us_ = now;

    // If ACK needs to be sent, do it here
    if (self->recv_ack_queued_ && !self->monitor_mode_) {
      self->recv_ack_queued_ = false;
      {
        InterruptLock lock;
        self->set_pin_output_low();
        delay_microseconds_safe(LOW_BIT_US);
        self->set_pin_input_high();
      }
    }
    return;
  }

  // Rising edge: compute pulse length → determines bit value
  auto pulse_duration = (now - self->last_falling_edge_us_);

  if (pulse_duration > START_BIT_MIN_US) {
    // Detected start bit → begin new frame
    self->receiver_state_ = ReceiverState::ReceivingByte;
    reset_state_variables_(self);
    self->recv_ack_queued_ = false;
    self->frame_receive_ = self->frames_queue_.back();
    return;
  } else if (pulse_duration < (HIGH_BIT_MIN_US / 4)) {
    // Very short pulse → noise/glitch
    return;
  }

  // Determine bit value based on duration
  bool value = (pulse_duration >= HIGH_BIT_MIN_US && pulse_duration <= HIGH_BIT_MAX_US);

  switch (self->receiver_state_) {

    case ReceiverState::ReceivingByte: {
      // Accumulate bits into current byte
      self->recv_byte_buffer_ = (self->recv_byte_buffer_ << 1) | (value & 0b1);
      self->recv_bit_counter_++;

      if (self->recv_bit_counter_ >= 8) {
        if (self->frame_receive_)
          self->frame_receive_->push_back(self->recv_byte_buffer_);

        self->recv_bit_counter_ = 0;
        self->recv_byte_buffer_ = 0;
        self->receiver_state_ = ReceiverState::WaitingForEOM;
      }
      break;
    }

    case ReceiverState::WaitingForEOM: {
      uint8_t dst = self->frame_receive_ ?
        (self->frame_receive_->front() & 0x0F) : 0xF;

      // If frame addressed to us → schedule ACK
      if (dst != 0xF && dst == self->address_)
        self->recv_ack_queued_ = true;

      bool is_eom = (value == 1);
      if (is_eom) {
        // Frame finished → pass frame to queue
        if (self->frame_receive_ && self->frame_receive_->size() > 0) {
          self->frames_queue_.push_back();
          self->frame_receive_ = nullptr;
        }
        reset_state_variables_(self);
      }

      self->receiver_state_ =
        (is_eom ? ReceiverState::WaitingForEOMAck : ReceiverState::WaitingForAck);
      break;
    }

    case ReceiverState::WaitingForAck:
      self->receiver_state_ = ReceiverState::ReceivingByte;
      break;

    case ReceiverState::WaitingForEOMAck:
      self->receiver_state_ = ReceiverState::Idle;
      break;

    default:
      break;
  }
}

// Reset temporary byte-accumulation state
void IRAM_ATTR HDMICEC::reset_state_variables_(HDMICEC *self) {
  self->recv_bit_counter_ = 0;
  self->recv_byte_buffer_ = 0x0;
}

}
}
