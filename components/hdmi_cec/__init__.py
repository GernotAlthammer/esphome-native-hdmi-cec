# Import ESPHome internal modules for code generation and validation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, automation

# Commonly used ESPHome constants
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID
)

# GitHub owner of this external component
CODEOWNERS = ["@Palakis"]

# Custom configuration keys for the component
CONF_PIN = "pin"
CONF_ADDRESS = "address"
CONF_PHYSICAL_ADDRESS = "physical_address"
CONF_PROMISCUOUS_MODE = "promiscuous_mode"
CONF_MONITOR_MODE = "monitor_mode"
CONF_DECODE_MESSAGES = "decode_messages"
CONF_OSD_NAME = "osd_name"
CONF_ON_MESSAGE = "on_message"

# Message filter / trigger options
CONF_SOURCE = "source"
CONF_DESTINATION = "destination"
CONF_OPCODE = "opcode"
CONF_DATA = "data"
CONF_PARENT = "parent"

# ---------------------------------------------------------
# Additional custom validation helpers
# ---------------------------------------------------------

def validate_data_array(value):
    """
    Validate the DATA field of a CEC message.
    DATA must be a list of byte values (0x00–0xFF).
    """
    if isinstance(value, list):
        return cv.Schema([cv.hex_uint8_t])(value)
    raise cv.Invalid("data must be a list of bytes")

def validate_osd_name(value):
    """
    Validate the CEC OSD (On-Screen Display) name.
    Requirements:
      - Must be a string
      - Length must be between 1 and 14 characters (CEC standard)
      - Only printable ASCII 0x20–0x7E is allowed
    """
    if not isinstance(value, str):
        raise cv.Invalid("Must be a string")
    if len(value) < 1:
        raise cv.Invalid("Must be a non-empty string")
    if len(value) > 14:
        raise cv.Invalid("Must not be more than 14-characters long")
    
    # Validate allowed ASCII range
    for char in value:
        if not 0x20 <= ord(char) < 0x7E:
            raise cv.Invalid(
                f"character '{char}' ({ord(char)}) is outside of the supported character range (0x20..0x7e)"
            )

    return value

# ---------------------------------------------------------
# Define the C++ classes used by the component
# ---------------------------------------------------------

# Creates a C++ namespace: namespace hdmi_cec { ... }
hdmi_cec_ns = cg.esphome_ns.namespace("hdmi_cec")

# Main HDMI-CEC component class
HDMICEC = hdmi_cec_ns.class_(
    "HDMICEC", cg.Component
)

# Trigger for incoming CEC messages
# Passes (source, destination, data_vector) to automations
MessageTrigger = hdmi_cec_ns.class_(
    "MessageTrigger",
    automation.Trigger.template(cg.uint8, cg.uint8, cg.std_vector.template(cg.uint8))
)

# Action for sending messages over CEC
SendAction = hdmi_cec_ns.class_(
    "SendAction", automation.Action
)

# ---------------------------------------------------------
# YAML configuration schema
# ---------------------------------------------------------

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(HDMICEC),

        # Output GPIO pin used to drive the CEC line
        cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,

        # Logical CEC device address (0–15)
        cv.Required(CONF_ADDRESS): cv.int_range(min=0, max=15),

        # Physical address according to the HDMI topology (e.g. 0x1000)
        cv.Required(CONF_PHYSICAL_ADDRESS): cv.uint16_t,

        # Mode settings
        cv.Optional(CONF_PROMISCUOUS_MODE, False): cv.boolean,
        cv.Optional(CONF_MONITOR_MODE, False): cv.boolean,
        cv.Optional(CONF_DECODE_MESSAGES, True): cv.boolean,

        # OSD name advertised by the CEC device
        cv.Optional(CONF_OSD_NAME, "esphome"): validate_osd_name,

        # Trigger automations when messages matching certain criteria are received
        cv.Optional(CONF_ON_MESSAGE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(MessageTrigger),
                cv.Optional(CONF_SOURCE): cv.int_range(min=0, max=15),
                cv.Optional(CONF_DESTINATION): cv.int_range(min=0, max=15),
                cv.Optional(CONF_OPCODE): cv.uint8_t,
                cv.Optional(CONF_DATA): validate_data_array
            }
        )
    }
)

# ---------------------------------------------------------
# Code generation: Converts YAML configuration to C++
# ---------------------------------------------------------

async def to_code(config):
    """
    Translate the YAML configuration into generated C++ initialization code.
    """

    # Optional preprocessor define enabling debug message decoding
    if config[CONF_DECODE_MESSAGES] == True:
        cg.add_define('USE_CEC_DECODER')

    # Create the C++ component instance
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Configure the GPIO pin used for the CEC bus
    cec_pin_ = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(cec_pin_))

    # Basic configuration parameters
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_physical_address(config[CONF_PHYSICAL_ADDRESS]))
    cg.add(var.set_promiscuous_mode(config[CONF_PROMISCUOUS_MODE]))
    cg.add(var.set_monitor_mode(config[CONF_MONITOR_MODE]))

    # Convert OSD name from Python string → ASCII bytes → std::vector<uint8_t>
    osd_name_bytes = bytes(config[CONF_OSD_NAME], 'ascii', 'ignore')
    osd_name_bytes = [x for x in osd_name_bytes]
    osd_name_bytes = cg.std_vector.template(cg.uint8)(osd_name_bytes)
    cg.add(var.set_osd_name_bytes(osd_name_bytes))

    # Build message triggers defined in YAML
    for conf in config.get(CONF_ON_MESSAGE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)

        # Optional message filter fields
        source = conf.get(CONF_SOURCE)
        if source is not None:
            cg.add(trigger.set_source(source))

        destination = conf.get(CONF_DESTINATION)
        if destination is not None:
            cg.add(trigger.set_destination(destination))

        opcode = conf.get(CONF_OPCODE)
        if opcode is not None:
            cg.add(trigger.set_opcode(opcode))

        data = conf.get(CONF_DATA)
        if data is not None:
            cg.add(trigger.set_data(data))

        # Register automation with the arguments it exposes
        await automation.build_automation(
            trigger,
            [
                (cg.uint8, "source"),
                (cg.uint8, "destination"),
                (cg.std_vector.template(cg.uint8), "data")
            ],
            conf
        )


# ---------------------------------------------------------
# Automation Action: hdmi_cec.send
# ---------------------------------------------------------

@automation.register_action(
    "hdmi_cec.send",
    SendAction,
    {
        cv.GenerateID(CONF_PARENT): cv.use_id(HDMICEC),

        # Optional source (defaults to component's logical address)
        cv.Optional(CONF_SOURCE): cv.templatable(cv.int_range(min=0, max=15)),

        # Destination device address
        cv.Required(CONF_DESTINATION): cv.templatable(cv.int_range(min=0, max=15)),

        # Raw data bytes to send
        cv.Required(CONF_DATA): cv.templatable(validate_data_array)
    }
)
async def send_action_to_code(config, action_id, template_args, args):
    """
    Generate the C++ implementation of the hdmi_cec.send action.
    Allows templatable expressions so values can depend on sensors,
    lambdas, globals, etc.
    """
    parent = await cg.get_variable(config[CONF_PARENT])

    # Create the action instance
    var = cg.new_Pvariable(action_id, template_args, parent)

    # Optional templatable source field
    source_template_ = await cg.templatable(config.get(CONF_SOURCE), args, cg.uint8)
    if source_template_ is not None:
        cg.add(var.set_source(source_template_))

    # Required destination field
    destination_template_ = await cg.templatable(config.get(CONF_DESTINATION), args, cg.uint8)
    cg.add(var.set_destination(destination_template_))

    # Templatable data vector
    data_vec_ = cg.std_vector.template(cg.uint8)
    data_template_ = await cg.templatable(config.get(CONF_DATA), args, data_vec_, data_vec_)
    cg.add(var.set_data(data_template_))

    return var
