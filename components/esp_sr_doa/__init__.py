import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

CODEOWNERS = ["@esphome"]
DEPENDENCIES = []

conf_ns = cg.esphome_ns.namespace("esp_sr_doa")
ESPSRDOA = conf_ns.class_("ESPSRDOA", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ESPSRDOA),
        cv.Optional("doa"): sensor.sensor_schema(
            unit_of_measurement="°",
            accuracy_decimals=1,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if "doa" in config:
        sens = await sensor.new_sensor(config["doa"])
        cg.add(var.set_doa_sensor(sens))

    # Use ESP-DSP for Custom GCC-PHAT
    cg.add_idf_component(
        name="esp-dsp",
        source="github://espressif/esp-dsp",
        ref="master",
    )
