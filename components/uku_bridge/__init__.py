import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID

uku_ns = cg.esphome_ns.namespace("uku_bridge")
UkuBridge = uku_ns.class_("UkuBridge", cg.Component)

CONF_CTRL_RX = "ctrl_rx_pin"
CONF_CTRL_TX = "ctrl_tx_pin"
CONF_CTRL_DE = "ctrl_de_pin"
CONF_PANEL_RX = "panel_rx_pin"
CONF_PANEL_TX = "panel_tx_pin"
CONF_PANEL_DE = "panel_de_pin"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UkuBridge),
        cv.Required(CONF_CTRL_RX): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_CTRL_TX): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_CTRL_DE): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_PANEL_RX): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_PANEL_TX): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_PANEL_DE): pins.internal_gpio_output_pin_number,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(
        var.set_pins(
            config[CONF_CTRL_RX],
            config[CONF_CTRL_TX],
            config[CONF_CTRL_DE],
            config[CONF_PANEL_RX],
            config[CONF_PANEL_TX],
            config[CONF_PANEL_DE],
        )
    )
