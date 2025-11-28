#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <array>

#include "hdmi_cec.h"

namespace esphome {
namespace hdmi_cec {

/**
 * @brief HDMI-CEC Frame Decoder
 *
 * This class converts a raw HDMI-CEC frame into a human-readable textual
 * description. It interprets the opcode, all associated operands, and the logical
 * source/destination addressing.
 *
 * Information for implementing this decoder is mainly derived from:
 *   • HDMI 1.3a Specification – "Supplement 1: Consumer Electronics Control (CEC)"
 *   • Additional ARC (Audio Return Channel) details from Linux v4l-utils
 *   • HDMI vendor ID mappings also sourced from v4l-utils
 *   • Short Audio Descriptor decoding information from EDID specifications
 */
class Decoder {
 public:
  /**
   * Construct a Decoder bound to a single CEC Frame.
   *
   * @param frame Reference to the CEC frame to decode.
   */
  Decoder(const Frame &frame) : frame_(frame), length_(0), offset_(2) {}

  /**
   * @brief Decode the frame into a full textual string.
   *
   * Produces a string in this form:
   *     "<Source> to <Destination>: <OpcodeName>[operand1][operand2]..."
   *
   * @return Printable description of the full decoded CEC frame.
   */
  std::string decode();

 protected:
  /**
   * @brief Look up the opcode name for a given opcode value.
   */
  const char *find_opcode_name(uint32_t opcode) const;

  /**
   * @brief Decode the CEC logical addressing (source → destination).
   *
   * Converts the 4-bit logical addresses into textual HDMI device roles such as:
   *     TV, Tuner1, PlaybackDev2, AudioSystem, etc.
   *
   * Broadcast frames return "All" as the destination address.
   */
  std::string address_decode() const;

  /**
   * -----------------------------------------------------------
   *   OPERAND DECODING
   * -----------------------------------------------------------
   *
   * CEC opcodes may have zero or more operands. Each operand can represent
   * different structured data types (physical address, power status, short audio
   * descriptor, timestamps, vendor IDs, and more).
   *
   * The HDMI-CEC spec defines a set of *operand types*. This generic template
   * dispatches to specialized template implementations (defined in the .cpp file).
   *
   * Returns:
   *   true  – more operands may still be decoded
   *   false – decoding must stop (likely end of frame)
   */
  template<uint32_t OPERANDS> bool do_operand();

  /**
   * -----------------------------------------------------------
   *   OPCODE TABLE
   * -----------------------------------------------------------
   *
   * HDMI-CEC 1.4 defines a fixed list of opcodes. For each opcode, we store:
   *   • its human-readable name
   *   • the operand decoding function to use
   *
   * The CecOpcodeTable maps:
   *     opcode → {name, decode_function}
   */
  using OperandDecode_f = bool (Decoder::*)();

  struct FrameType {
    const char *name;                ///< Name of the CEC command
    const OperandDecode_f decode_f;  ///< Pointer to the operand-decode handler
  };

  using CecOpcodeTable = const std::map<uint8_t, FrameType>;
  static const CecOpcodeTable cec_opcode_table;

  // Underlying frame being decoded
  const Frame &frame_;

  /**
   * Temporary output buffer:
   *
   * While decoding operands, we build a string into `line_`. The final decode()
   * concatenates it with the header and opcode name.
   */
  std::array<char, 256> line_;
  unsigned int length_;  ///< Current length of output in line_
  unsigned int offset_;  ///< Current byte offset into frame payload (operands begin at index 2)

  /**
   * -----------------------------------------------------------
   *   OPERAND TYPE ENUMERATION
   * -----------------------------------------------------------
   *
   * Every operand type defined in the HDMI-CEC standard appears here.
   * Many opcodes use combinations of these operand types.
   *
   * Each entry corresponds to a specific specialized `do_operand<>` template.
   */
  enum Operand : uint8_t {
    None,
    AbortReason,
    AnalogBroadcastType,
    AnalogFrequency,
    AsciiDigit,
    Ascii,
    AudioFormat,
    AudioRate,
    AudioStatus,
    Boolean,
    BroadcastSystem,
    CecVersion,
    ChannelIdentifier,
    DeckControlMode,
    DeckInfo,
    DeviceType,
    DigitalServiceIdentification,
    DisplayControl,
    Duration,
    ExternalPhysicalAddress,
    ExternalPlug,
    ExternalSourceSpecifier,
    Hour,
    FeatureOpcode,
    Language,
    MenuRequestType,
    MenuState,
    Minute,
    NewAddress,
    OriginalAddress,
    OsdName,
    OsdString = OsdName,
    PhysicalAddress,
    PlayMode,
    PowerStatus,
    ProgramTitleString,
    RecordSource,
    RecordStatusInfo,
    RecordingSequence,
    ShortAudioDescriptor,
    StatusRequest,
    StartDateTime,
    SystemAudioStatus,
    Time,
    TimerClearedStatusData,
    TimerStatusData,
    TunerDeviceInfo,
    UIBroadcastType,
    UICommand,
    UIFunctionMedia,
    UIFunctionSelectAVInput,
    UIFunctionSelectAudioInput,
    UISoundPresentationControl,
    VendorId,
    VendorSpecificData,
    VendorSpecificRCCode,
  };

  /**
   * Some opcodes expect multiple operands in sequence. These helper functions
   * encode up to 3 operand types into a single uint32, so the template system
   * can unpack and decode each operand in order.
   */
  static constexpr uint32_t Two(uint32_t first, uint32_t second) {
    return first | (second << 8);
  }
  static constexpr uint32_t Three(uint32_t first, uint32_t second, uint32_t third) {
    return first | (second << 8) | (third << 16);
  }

  /**
   * Helper used by operand decoding to append human-readable text.
   *
   * @param word         Text to append inside square brackets [word]
   * @param offset_incr  How many bytes to advance the frame offset (default = 1)
   *
   * Updates:
   *   - line_[] buffer
   *   - length_
   *   - offset_
   *
   * Returns whether decoding may continue.
   */
  bool append_operand(const char *word, uint8_t offset_incr = 1);

  /**
   * Overload for table-based operands (e.g., enums represented as uint8).
   *
   * Selects the string associated with the numeric operand value, or "?" if out
   * of bounds.
   */
  template<uint32_t N_STRINGS>
  bool append_operand(const std::array<const char *, N_STRINGS> &strings) {
    uint32_t operand_value = frame_[offset_];
    const char *s = (operand_value < N_STRINGS) ? strings[operand_value] : "?";
    return append_operand(s);
  }

  /**
   * Lookup tables for specific operand types
   */
  static const std::array<const char *, 0x77> UI_Commands;       ///< UI command names
  static const std::array<const char *, 0x11> audio_formats;     ///< EDID short audio format names
  static const std::array<const char *, 8> audio_samplerates;    ///< Audio sample rate labels
  static const std::map<uint32_t, const char *> vendor_ids;      ///< HDMI vendor ID → brand
};

}  // namespace hdmi_cec
}  // namespace esphome
