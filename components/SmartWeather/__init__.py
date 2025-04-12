import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor
from esphome.const import (
    CONF_ID,
    CONF_SIGNAL_STRENGTH,
    CONF_TEMPERATURE,
    CONF_HUMIDITY,
    CONF_WIND_SPEED,
    CONF_WIND_DIRECTION_DEGREES,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_DECIBEL,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_DEGREES,
    UNIT_MILLIMETER,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_WIND_SPEED,
    DEVICE_CLASS_PRECIPITATION,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_SIGNAL,
    ICON_BATTERY,
    ICON_THERMOMETER,
    ICON_WATER_PERCENT,
    ICON_WEATHER_WINDY,
    ICON_SIGN_DIRECTION,
)

UNIT_METER_PER_SECOND = "m/s"
CONF_STATION_ID = "station_id"
CONF_BATTERY = "battery"
CONF_TOTAL_ACC = "total_acc"


smartweather_ns = cg.esphome_ns.namespace("smartweather")
SmartWeatherComponent = smartweather_ns.class_("SmartWeatherComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SmartWeatherComponent),
        cv.Optional(CONF_STATION_ID): sensor.sensor_schema(),
        cv.Optional(CONF_SIGNAL_STRENGTH): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon=ICON_SIGNAL,
        ),
        cv.Optional(CONF_BATTERY): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY,
            icon=ICON_BATTERY,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_THERMOMETER,
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_WATER_PERCENT,
        ),        
        cv.Optional(CONF_WIND_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER_PER_SECOND,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_WEATHER_WINDY,
        ),
        cv.Optional(CONF_WIND_DIRECTION_DEGREES): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=0,
            icon=ICON_SIGN_DIRECTION,
        ),
        cv.Optional(CONF_TOTAL_ACC): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_PRECIPITATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_BATTERY in config:
        sens = await sensor.new(config[CONF_BATTERY])
        cg.add(var.set_wind_speed(sens))

    if CONF_SIGNAL_STRENGTH in config:
        conf = config[CONF_SIGNAL_STRENGTH]
        sens = await sensor.new(conf)
        cg.add(var.set_wind_speed(sens))
        
    if CONF_WIND_SPEED in config:
        conf = config[CONF_WIND_SPEED]
        sens = await sensor.new(conf)
        cg.add(var.set_wind_speed(sens))

    if CONF_WIND_SPEED in config:
        conf = config[CONF_WIND_SPEED]
        sens = await sensor.new(conf)
        cg.add(var.set_wind_speed(sens))

    if CONF_WIND_SPEED in config:
        conf = config[CONF_WIND_SPEED]
        sens = await sensor.new(conf)
        cg.add(var.set_wind_speed(sens))

    if CONF_WIND_DIRECTION_DEGREES in config:
        conf = config[CONF_WIND_DIRECTION_DEGREES]
        sens = await sensor.new(conf)
        cg.add(var.set_wind_direction_degrees(sens))