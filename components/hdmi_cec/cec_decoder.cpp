#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <array>

#include "hdmi_cec.h"
#include "cec_decoder.h"

namespace esphome {
namespace hdmi_cec {

/**
 * Lookup table for UI Commands defined by HDMI-CEC.
 * Index = UI command code (0x00–0x76)
 * Value = Human-readable command name
 *
 * This table allows conversion of UI command opcodes into descriptive text
 * when decoding <User Control Pressed> messages.
 */
const std::array<const char *, 0x77> Decoder::UI_Commands = {
    /* 0x00 = */ "Select",
    "Up",
    "Down",
    "Left",
    "Right",
    "Right-Up",
    "Right-Down",
    "Left-Up",
    /* 0x08 = */ "Left-Down",
    "Root Menu",
    "Setup Menu",
    "Contents Menu",
    "Favorite Menu",
    "Exit",
    "Reserved",
    "Reserved",
    /* 0x10 = */ "Media Top Menu",
    "Media Context-sensitive Menu",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    /* 0x18 = */ "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Number Entry Mode",
    "11",
    "12",
    /* 0x20 = */ "0",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    /* 0x28 = */ "8",
    "9",
    "Dot",
    "Enter",
    "Clear",
    "Reserved",
    "Reserved",
    "Next Favorite",
    /* 0x30 = */ "Channel Up",
    "Channel Down",
    "Previous Channel",
    "Sound Select",
    "Input Select",
    "Display Information",
    "Help",
    "Page Up",
    /* 0x38 = */ "Page Down",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    /* 0x40 = */ "Power",
    "Volume Up",
    "Volume Down",
    "Mute",
    "Play",
    "Stop",
    "Pause",
    "Record",
    /* 0x48 = */ "Rewind",
    "Fast forward",
    "Eject",
    "Forward",
    "Backward",
    "Stop-Record",
    "Pause-Record",
    "Reserved",
    /* 0x50 = */ "Angle",
    "Sub picture",
    "Video on Demand",
    "Electronic Program Guide",
    "Timer Programming",
    "Initial Configuration",
    "Select Broadcast Type",
    "Select Sound Presentation",
    /* 0x58 = */ "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    /* 0x60 = */ "Play Function",
    "Pause-Play Function",
    "Record Function",
    "Pause-Record Function",
    "Stop Function",
    "Mute Function",
    "Restore Volume Function",
    "Tune Function",
    /* 0x68 = */ "Select Media Function",
    "Select A/V Input Function",
    "Select Audio Input Function",
    "Power Toggle Function",
    "Power Off Function",
    "Power On Function",
    "Reserved",
    "Reserved",
    /* 0x70 = */ "Reserved",
    "F1 (Blue)",
    "F2 (Red)",
    "F3 (Green)",
    "F4 (Yellow)",
    "F5",
    "Data"};

/**
 * Lookup tables for decoding audio descriptors contained in Short Audio Descriptor (SAD) fields.
 * These tables reference the EDID standard.
 */
const std::array<const char *, 0x11> Decoder::audio_formats = {
    "reserved", "LPCM", "AC3",    "MPEG-1",           "MP3",       "MPEG-2",  "AAC",
    "DTS",      "ATRAC","DSD",    "DD+",               "DTS-HD",    "MAT/Dolby TrueHD",
    "DST Audio","WMA Pro","Extension?"};

const std::array<const char *, 8> Decoder::audio_samplerates = {
    "32", "44.1", "48", "88", "96", "176", "192", "Reserved"};


/**
 * Generic operand decoding dispatcher.
 *
 * The template parameter OPERANDS encodes one or multiple operand types,
 * each being one byte wide (lower byte = next operand).
 *
 * This recursive method handles:
 *   - operand lists (multiple operands packed in a 32-bit integer)
 *   - unknown and variable-length operands
 */
template<uint32_t OPERANDS>
bool Decoder::do_operand() {
  if (OPERANDS <= 0xFF) {
    // Generic fallback: operand of unknown type or length
    return append_operand(".");
  } else {
    // Decode lowest byte, then recurse
    return do_operand<OPERANDS & 0xFF>() && do_operand<(OPERANDS >> 8u)>();
  }
}

/**
 * Below are all specialized operand decoders.
 * Each template specialization implements the decoding of a specific operand type.
 * These operand types correspond to HDMI-CEC specification fields.
 */

template<>
bool Decoder::do_operand<Decoder::None>() {
  // No operand needed for this opcode
  return append_operand("");
}

template<>
bool Decoder::do_operand<Decoder::AbortReason>() {
  static const std::array<const char *, 7> names = {
      "Unrecognized opcode",
      "Not in correct mode to respond",
      "Cannot provide source",
      "Invalid operand",
      "Refused",
      "Unable to determine"
  };
  return append_operand<names.size()>(names);
}

template<>
bool Decoder::do_operand<Decoder::AudioFormat>() {
  // This operand type consists of a sequence of audio formats
  bool ok = true;
  while (ok && (offset_ < frame_.size())) {
    ok = append_operand<audio_formats.size()>(audio_formats);
  }
  return ok;
}

template<>
bool Decoder::do_operand<Decoder::AudioStatus>() {
  // Audio status consists of mute flag + volume level
  char line[20];
  if (offset_ < frame_.size()) {
    uint8_t field = frame_[offset_];
    std::sprintf(line, "Mute=%d,Vol=%02X", (field >> 7), (field & 0x7F));
    return append_operand(line);
  }
  return append_operand("?");
}

template<>
bool Decoder::do_operand<Decoder::DeviceType>() {
  static const std::array<const char *, 9> names = {
      "TV", "Recording Device", "Reserved", "Tuner",
      "Playback Device", "Audio System", "Pure CEC Switch", "Video Processor"
  };
  return append_operand<names.size()>(names);
}

template<>
bool Decoder::do_operand<Decoder::DisplayControl>() {
  static const std::array<const char *, 9> names = {
      "Default Time", "Until cleared", "Clear previous", "Reserved"
  };
  return append_operand<names.size()>(names);
}

template<>
bool Decoder::do_operand<Decoder::FeatureOpcode>() {
  if (offset_ >= frame_.size()) {
    return false;
  }
  uint8_t opcode = frame_[offset_];
  return append_operand(find_opcode_name(opcode));
}

template<>
bool Decoder::do_operand<Decoder::OsdString>() {
  // Copy ASCII OSD string from frame into buffer
  char line[20];
  std::snprintf(line, frame_.size() - offset_ + 1, "%s",
                reinterpret_cast<const char *>(frame_.data() + offset_));
  offset_ = frame_.size();  // Consumes remaining frame bytes
  return append_operand(line);
}

template<>
bool Decoder::do_operand<Decoder::PhysicalAddress>() {
  // Special rule: System Audio Mode Request (0x70) may omit this operand
  if (frame_.at(1) == 0x70 && offset_ >= frame_.size()) {
    return append_operand("Off");
  }
  if (offset_ + 1 >= frame_.size()) {
    return append_operand("?", 2);
  }
  char line[12];
  std::sprintf(line, "%1x.%1x.%1x.%1x",
               (frame_[offset_] >> 4) & 0xF,
               frame_[offset_] & 0xF,
               (frame_[offset_ + 1] >> 4) & 0xF,
               frame_[offset_ + 1] & 0xF);
  return append_operand(line, 2);
}

template<>
bool Decoder::do_operand<Decoder::PowerStatus>() {
  static const std::array<const char *, 5> names = {
      "On", "Standby", "Standby->On", "On->Standby"
  };
  return append_operand<names.size()>(names);
}

template<>
bool Decoder::do_operand<Decoder::ShortAudioDescriptor>() {
  /**
   * A Short Audio Descriptor (SAD) consists of 3 bytes and may repeat.
   * This loop decodes all sequential SADs in the frame.
   */
  std::array<char, 100> line;
  uint32_t pos = 0;
  bool ok = true;

  while (ok && (offset_ + 2 < frame_.size())) {
    const uint8_t *descriptor = frame_.data() + offset_;
    uint8_t format = (descriptor[0] >> 3) & 0x0F;
    pos = std::sprintf(&line[0], "%s", audio_formats[format]);

    pos += std::sprintf(&line[pos], ",num_channels=%d", (descriptor[0] & 0x07));

    // Decode supported audio sample rates
    uint8_t rates = descriptor[1];
    for (int bit = 0; rates; bit++, rates >>= 1) {
      if (rates & 0x1) {
        pos += std::sprintf(&line[pos], ",%skHz", audio_samplerates[bit]);
      }
    }

    // LPCM-specific: sample bit widths
    if (format == 1) {
      uint8_t widths = descriptor[2] & 0x07;
      for (int i = 0; widths; i++, widths >>= 1) {
        if (widths & 0x1) {
          pos += std::sprintf(&line[pos], ",%dbits", (16 + 4 * i));
        }
      }
    }

    ok = append_operand(&line[0], 3);
  }

  // TODO: Additional extensions not yet implemented
  return ok;
}

template<>
bool Decoder::do_operand<Decoder::SystemAudioStatus>() {
  static const std::array<const char *, 3> names = {"Off", "On"};
  return append_operand<names.size()>(names);
}

template<>
bool Decoder::do_operand<Decoder::UICommand>() {
  uint8_t command = frame_[offset_];

  // First decode the main UI command
  bool ok = append_operand<UI_Commands.size()>(UI_Commands);
  if (!ok) return false;

  // Some UI Commands include a follow-up parameter
  switch (command) {
    case 0x56: return do_operand<UIBroadcastType>();
    case 0x57: return do_operand<UISoundPresentationControl>();
    case 0x60: return do_operand<PlayMode>();
    case 0x67: return do_operand<ChannelIdentifier>();
    case 0x68: return do_operand<UIFunctionMedia>();
    case 0x69: return do_operand<UIFunctionSelectAVInput>();
    case 0x6A: return do_operand<UIFunctionSelectAudioInput>();
    default:   return ok;
  }
}

template<>
bool Decoder::do_operand<Decoder::VendorId>() {
  if (offset_ + 2 >= frame_.size()) {
    return append_operand("?", 3);
  }

  // Vendor ID is a 24-bit identifier
  uint32_t id =
      (uint32_t)(frame_[offset_]) << 16 |
      (uint32_t)(frame_[offset_ + 1]) << 8 |
      frame_[offset_ + 2];

  auto it = vendor_ids.find(id);
  if (it == vendor_ids.end()) {
    // Unknown vendor — print raw hex ID
    char line[12];
    sprintf(line, "ID=%06x", id);
    return append_operand(line, 3);
  }

  return append_operand(it->second, 3);
}

template<>
bool Decoder::do_operand<Decoder::CecVersion>() {
  static const std::array<const char *, 9> names = {
      "?", "1.2", "1.2a", "1.3", "1.3a", "1.4", "2.0", "2.x", "2.x"
  };
  return append_operand<names.size()>(names);
}

/**
 * Table mapping CEC opcodes to:
 *  - human-readable message names
 *  - the operand decoding function
 *
 * This table forms the core of the decoding logic.
 */
const Decoder::CecOpcodeTable Decoder::cec_opcode_table = {
    // opcode,   {name,                 decode_function}
    {0x04, {"Image View On",           &Decoder::do_operand<None>}},
    {0x00, {"Feature Abort",           &Decoder::do_operand<Two(FeatureOpcode, AbortReason)>>}},
    {0x0D, {"Text View On",            &Decoder::do_operand<None>}},
    {0x82, {"Active Source",           &Decoder::do_operand<PhysicalAddress>}},
    {0x9D, {"Inactive Source",         &Decoder::do_operand<PhysicalAddress>}},
    {0x85, {"Request Active Source",   &Decoder::do_operand<None>}},
    {0x80, {"Routing Change",          &Decoder::do_operand<Two(PhysicalAddress, PhysicalAddress)>>}},
    {0x81, {"Routing Information",     &Decoder::do_operand<PhysicalAddress>}},
    {0x86, {"Set Stream Path",         &Decoder::do_operand<PhysicalAddress>}},
    {0x36, {"Standby",                 &Decoder::do_operand<None>}},
    ...
    {0xF8, {"CDC Message",             &Decoder::do_operand<None>}}
};

/**
 * Known HDMI-CEC Vendor IDs mapped to human-readable names.
 * Used by the <Device Vendor ID> operand decoder.
 */
const std::map<uint32_t, const char *> Decoder::vendor_ids = {
    {0x000039, "Toshiba"}, {0x0000F0, "Samsung"}, {0x0005CD, "Denon"},
    {0x000678, "Marantz"}, {0x000982, "Loewe"},  {0x0009B0, "Onkyo"},
    ...
    {0x9C645E, "Harman Kardon"}
};

/**
 * Decodes the source/destination portion of the CEC message.
 * This forms the first part of the output string.
 */
std::string Decoder::address_decode() const {
  static const std::array<const char *, 16> names = {
      "TV", "RecordingDev1", "RecordingDev2", "Tuner1",
      "PlaybackDev1", "AudioSystem", "Tuner2", "Tuner3",
      "PlaybackDev2", "RecordingDev3", "Tuner4", "PlaybackDev3",
      "Reserved", "Reserved", "SpecificUse", "Unregistered"
  };

  const char* dest = (frame_.is_broadcast())
                        ? "All"
                        : names[frame_.destination_addr()];
  return std::string(names[frame_.initiator_addr()]) + " to " + dest + ": ";
}

/**
 * Lookup helper that finds the name of an opcode.
 * Returns "?" if the opcode does not exist in the table.
 */
const char *Decoder::find_opcode_name(uint32_t opcode) const {
  auto it = cec_opcode_table.find(opcode);
  if (it == cec_opcode_table.end()) {
    return "?";
  }
  return it->second.name;
}

/**
 * Utility function used by all operand decoders.
 * Appends a formatted operand string to the decoder's output buffer.
 *
 * @param word         String to append inside brackets
 * @param offset_incr  Number of frame bytes consumed by the operand
 * @return true if more decoding can proceed; false if the buffer or frame ends
 */
bool Decoder::append_operand(const char *word, uint8_t offset_incr) {
  length_ += snprintf(&line_[length_], (line_.size() - length_), "[%s]", word);
  offset_ += offset_incr;
  return (length_ < line_.size()) && (offset_ < frame_.size());
}

/**
 * Main entry point to decode an entire CEC frame into a human-readable string.
 */
std::string Decoder::decode() {
  // First decode the logical address fields
  std::string result = address_decode();

  // Frames with only a header (length ≤ 1) are "Ping" frames
  if (frame_.size() <= 1) {
    return result + "Ping";
  }

  // Look up opcode
  auto it = cec_opcode_table.find(frame_.opcode());
  if (it == cec_opcode_table.end()) {
    return result + std::string("<?>");  // Unknown opcode
  }

  // Append opcode name
  result += std::string("<") + it->second.name + ">";

  // Prepare operand decoding buffer
  length_ = 0;
  line_[length_] = 0;
  offset_ = 2;  // Operands start at byte index 2

  // Call the appropriate operand decoder
  OperandDecode_f f = it->second.decode_f;
  (this->*f)();

  // Append operand text
  result += &line_[0];
  return result;
}

}  // namespace hdmi_cec
}  // namespace esphome
