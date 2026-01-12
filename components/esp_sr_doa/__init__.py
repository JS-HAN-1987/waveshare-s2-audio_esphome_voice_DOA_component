import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.components.esp32 import add_idf_component
from esphome.const import CONF_ID

CODEOWNERS = ["@antigravity"]
DEPENDENCIES = ["esp32"]

esp_sr_doa_ns = cg.esphome_ns.namespace("esp_sr_doa")
ESPSRDOA = esp_sr_doa_ns.class_("ESPSRDOA", cg.Component)

CONF_DOA_SENSOR = "doa_sensor"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ESPSRDOA),
    cv.Optional(CONF_DOA_SENSOR): sensor.sensor_schema(
        icon="mdi:compass-outline",
        unit_of_measurement="°",
        accuracy_decimals=1,
    ),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_DOA_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_DOA_SENSOR])
        cg.add(var.set_doa_sensor(sens))

    # Add ESP-SR as an IDF component (DOA functions only)
    add_idf_component(
        name="esp-sr",
        repo="https://github.com/espressif/esp-sr.git",
        ref="master",
    )
