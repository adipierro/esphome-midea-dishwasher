import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID
from esphome.core import CORE, Define, coroutine_with_priority

CODEOWNERS = ["@0xgiddi"]
DEPENDENCIES = ["uart"]
MULTI_CONF = False

CONF_TX_UART = "tx_uart"
CONF_RX_UART = "rx_uart"
CONF_DISABLE_UART_WAKE_LOOP_ON_RX = "disable_uart_wake_loop_on_rx"


@coroutine_with_priority(-1001.0)
async def disable_uart_wake_loop_on_rx():
    """Remove ESPHome's networking-enabled UART RX wake callback."""
    CORE.defines.discard(Define("USE_UART_WAKE_LOOP_ON_RX"))


midea_dishwasher_ns = cg.esphome_ns.namespace("midea_dishwasher")
MideaDishwasher = midea_dishwasher_ns.class_("MideaDishwasher", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MideaDishwasher),
    cv.Required(CONF_TX_UART): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_RX_UART): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_DISABLE_UART_WAKE_LOOP_ON_RX, default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    tx_uart = await cg.get_variable(config[CONF_TX_UART])
    rx_uart = await cg.get_variable(config[CONF_RX_UART])
    var = cg.new_Pvariable(config[CONF_ID], tx_uart, rx_uart)
    await cg.register_component(var, config)
    if config[CONF_DISABLE_UART_WAKE_LOOP_ON_RX]:
        # UART adds the define at FINAL priority (-1000); remove it one step later.
        CORE.add_job(disable_uart_wake_loop_on_rx)
