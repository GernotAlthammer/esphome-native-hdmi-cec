#pragma once

#include <array>
#include <vector>
#include <atomic>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace hdmi_cec {

/**
 * A CEC Frame is represented as a vector of bytes.
 * The HDMI-CEC protocol always begins with a header byte:
 *   - Upper nibble: initiator (source) address
 *   - Lower nibble: destination address
 * The second byte is the opcode, followed by optional operands.
 */
class Frame : public std::vector<uint8_t> {
 public:
  Frame() = default;

  /**
   * Constructor: builds a CEC frame from source address, destination address,
   * and a list of payload bytes. The first byte becomes (initiator << 4 | destination),
   * followed by the provided payload (opcode + operands).
   */
  Frame(uint8_t initiator_addr, uint8_t target_addr, const std::vector<uint8_t> &payload);

  /// Returns the 4-bit initiator address extracted from the header byte.
  uint8_t initiator_addr() const { return (this->at(0) >> 4) & 0xf; }

  /// Returns the 4-bit destination address extracted from the header byte.
  uint8_t destination_addr() const { return this->at(0) & 0xf; }

  /// Returns the opcode if available; otherwise returns 0.
  uint8_t opcode() const { return (this->size() >= 2) ? this->at(1) : 0; }

  /// True if the destination address is the broadcast address (0xF).
  bool is_broadcast() const { return this->destination_addr() == 0xf; }

  /**
   * Converts the frame into a human-readable string, optionally skipping
   * decoder-level interpretation.
   */
  std::string to_string(bool skip_decode = 0) const;

  /// The HDMI-CEC standard defines a maximum frame size of 16 bytes.
  constexpr static int MAX_LENGTH = 16;
};

/**
 * Receiver states used by the internal CEC decoding state machine.
 * They represent the current stage of receiving a CEC byte or bit.
 */
enum class ReceiverState : uint8_t {
  Idle = 0,         // No active reception
  ReceivingByte = 2,  // Currently decoding bits of a byte
  WaitingForEOM = 3,   // Waiting for End-Of-Message bit
  WaitingForAck = 4,   // Waiting for ACK from receiving device
  WaitingForEOMAck = 5 // Waiting for ACK specifically after the EOM bit
};

/**
 * Result codes returned by the send() operation.
 */
enum class SendResult : uint8_t {
  Success = 0,     // Frame sent successfully
  BusCollision = 1, // Another device pulled the line low during transmit
  NoAck = 2,        // Destination device did not acknowledge the frame
};

/**
 * A lock-free ring buffer for storing CEC frames.
 *
 * DESIGN NOTES:
 *  - Uses std::atomic for index pointers to support a single producer thread
 *    and a single consumer thread.
 *  - Memory for all frames is allocated up front, meaning no dynamic allocation
 *    during interrupt handling.
 *  - Buffer size is MAX_FRAMES_QUEUED + 1 internally to distinguish
 *    full vs. empty states.
 */
template <unsigned int SIZE>
class FrameRingBuffer {
 public:
  FrameRingBuffer()
  : front_inx_{0}
  , back_inx_{0}
  , store_{} {
    // Preallocate frames so no allocation occurs in ISR context.
    for (auto& t : store_) {
      t = new Frame;
      t->reserve(Frame::MAX_LENGTH);
    }
  }

  ~FrameRingBuffer() {
    for (auto& t : store_) {
      delete t;
    }
  }

  /**
   * Returns a pointer to the oldest queued frame.
   * Returns nullptr if the buffer is empty.
   */
  Frame* front() const { return is_empty() ? nullptr : store_[front_inx_]; }

  /// Moves the front pointer forward, effectively discarding the front frame.
  void push_front() { cyclic_incr(front_inx_); }

  /**
   * Returns a pointer to a free frame for writing new data.
   * Returns nullptr if the buffer is full.
   */
  Frame* back() const { return is_full() ? nullptr : (store_[back_inx_]->clear(), store_[back_inx_]); }

  /// Moves the back pointer forward to finalize a new frame insert.
  void push_back() { cyclic_incr(back_inx_); }

  bool is_empty() const { return count() == 0; }
  bool is_full()  const { return count() == SIZE; }

  /// Resets the ring buffer (does not clear Frame contents).
  void reset() { front_inx_ = 0; back_inx_ = 0; }

 protected:
  using Index = std::atomic<unsigned int>;

  /**
   * Computes the number of frames currently stored.
   * Using unsigned arithmetic makes wrap-around safe.
   */
  int count() const {
    int n = (int)(back_inx_ - front_inx_);
    if (n < 0) n += SIZE + 1;
    return n;
  }

  /// Increments an index with wrap-around behavior.
  void cyclic_incr(Index &inx) { inx = (inx == SIZE) ? 0 : (inx + 1); }

  Index front_inx_;  // Index of next frame to be consumed
  Index back_inx_;   // Index of next free frame to write into

  /**
   * Store contains SIZE + 1 frame pointers to distinguish
   * between full and empty buffer states.
   */
  std::array<Frame*, SIZE + 1> store_;
};

class MessageTrigger;

class HDMICEC : public Component {
public:
  /// Assign the GPIO pin used for sending/receiving CEC signals.
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }

  /// Logical CEC device address (0x0 – 0xF).
  void set_address(uint8_t address) { address_ = address; }
  uint8_t address() { return address_; }

  /// Sets the physical address (n.n.n.n), encoded as 16-bit.
  void set_physical_address(uint16_t physical_address) { physical_address_ = physical_address; }

  /// If true: accept all frames regardless of destination.
  void set_promiscuous_mode(bool promiscuous_mode) { promiscuous_mode_ = promiscuous_mode; }

  /// If true: do not ACK, do not reply—monitor only.
  void set_monitor_mode(bool monitor_mode) { monitor_mode_ = monitor_mode; }

  /// Local device name in UTF-8 bytes used for OSD (On-Screen Display) Name messages.
  void set_osd_name_bytes(const std::vector<uint8_t> &osd_name_bytes) { osd_name_bytes_ = osd_name_bytes; }

  /// Add message triggers (automations) that execute when matching CEC frames are received.
  void add_message_trigger(MessageTrigger *trigger) { message_triggers_.push_back(trigger); }

  /**
   * Send a CEC message with the given source, destination, and payload.
   * Handles arbitration, ACK, timing, and retries according to CEC spec.
   */
  bool send(uint8_t source, uint8_t destination, const std::vector<uint8_t> &data_bytes);

  // ESPHome component interface
  float get_setup_priority() { return esphome::setup_priority::HARDWARE; }
  void setup() override;
  void dump_config() override;
  void loop() override;

protected:
  /// GPIO ISR callback: handles bit-level input timing and decoding.
  static void gpio_intr_(HDMICEC *self);

  /// Resets the internal receiver state machine variables.
  static void reset_state_variables_(HDMICEC *self);

  /// Handles built-in automatic responses (e.g., report device info, OSD name).
  void try_builtin_handler_(uint8_t source, uint8_t destination, const std::vector<uint8_t> &data);

  /// Sends a complete CEC frame and returns the result.
  SendResult send_frame_(const Frame &frame, bool is_broadcast);

  /// Sends a CEC start bit (3.7 ms low, 0.8 ms high).
  bool send_start_bit_();

  /// Sends a single logical bit according to CEC line timing.
  void send_bit_(bool bit_value);

  /// Drives the line high and checks for bus collision.
  bool send_high_and_test_();

  /// Configure pin as input with internal pull-up to read CEC line.
  void set_pin_input_high();

  /// Drive line low (CEC is active-low).
  void set_pin_output_low();

  constexpr static int MAX_FRAMES_QUEUED = 4;

  InternalGPIOPin *pin_;   // Main application pin
  ISRInternalGPIOPin isr_pin_; // ISR-safe version of the pin

  uint8_t address_;          // Logical device address
  uint16_t physical_address_; // Physical routing address like 1.0.0.0

  bool promiscuous_mode_;    // Accept frames for any destination
  bool monitor_mode_;        // Do not reply to CEC messages

  std::vector<uint8_t> osd_name_bytes_; // Device name advertised in CEC messages
  std::vector<MessageTrigger*> message_triggers_; // Automations

  // Receiver state variables
  bool last_level_ = true;
  uint32_t last_falling_edge_us_ = 0;
  uint32_t last_sent_us_ = 0;
  ReceiverState receiver_state_;
  uint8_t recv_bit_counter_ = 0;
  uint8_t recv_byte_buffer_ = 0;
  Frame *frame_receive_ = nullptr;

  // Queue of received frames
  FrameRingBuffer<MAX_FRAMES_QUEUED> frames_queue_;

  bool recv_ack_queued_ = false;

  Mutex send_mutex_; // Prevents concurrent send attempts
};

/**
 * Automation trigger for CEC messages.
 * Allows filtering by:
 *  - source address
 *  - destination address
 *  - opcode
 *  - full data payload
 */
class MessageTrigger : public Trigger<uint8_t, uint8_t, std::vector<uint8_t>> {
  friend class HDMICEC;

public:
  explicit MessageTrigger(HDMICEC *parent) { parent->add_message_trigger(this); };

  void set_source(uint8_t source) { source_ = source; };
  void set_destination(uint8_t destination) { destination_ = destination; };
  void set_opcode(uint8_t opcode) { opcode_ = opcode; };
  void set_data(const std::vector<uint8_t> &data) { data_ = data; };

protected:
  optional<uint8_t> source_;
  optional<uint8_t> destination_;
  optional<uint8_t> opcode_;
  optional<std::vector<uint8_t>> data_;
};

/**
 * Action for sending CEC messages from ESPHome automations.
 * Uses templatable values to allow dynamic evaluation at runtime.
 */
template<typename... Ts> class SendAction : public Action<Ts...> {
public:
  SendAction(HDMICEC *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, source)
  TEMPLATABLE_VALUE(uint8_t, destination)
  TEMPLATABLE_VALUE(std::vector<uint8_t>, data)

  void play(const Ts&... x) override {
    auto source_address = source_.has_value() ? source_.value(x...) : parent_->address();
    auto destination_address = destination_.value(x...);
    auto data = data_.value(x...);
    parent_->send(source_address, destination_address, data);
  }

protected:
  HDMICEC *parent_;
};

}  // namespace hdmi_cec
}  // namespace esphome
