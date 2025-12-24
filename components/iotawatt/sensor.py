import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_TYPE,
    CONF_MODEL,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_VOLT,
    UNIT_WATT,
)

CONF_INPUTS = "inputs"
CONF_CHANNEL = "channel"
CONF_CAL = "cal"
CONF_PHASE = "phase"
CONF_TURNS = "turns"
CONF_REVERSE = "reverse"
CONF_POWER_FACTOR = "power_factor"
CONF_VPHASE = "vphase"
CONF_DOUBLE = "double"

CONF_VOLTAGE_SENSOR = "voltage"
CONF_FREQUENCY_SENSOR = "frequency"
CONF_POWER_SENSOR = "power"
CONF_CURRENT_SENSOR = "current"

CONF_CLK_PIN = "clk_pin"
CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_CS_PIN_0 = "cs_pin_0"
CONF_CS_PIN_1 = "cs_pin_1"

iotawatt_ns = cg.esphome_ns.namespace("iotawatt")
IoTaWattComponent = iotawatt_ns.class_("IoTaWattComponent", cg.Component)

VOLTAGE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_VOLTAGE,
    state_class=STATE_CLASS_MEASUREMENT,
)

POWER_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_WATT,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_POWER,
    state_class=STATE_CLASS_MEASUREMENT,
)

FREQUENCY_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_HERTZ,
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_FREQUENCY,
    state_class=STATE_CLASS_MEASUREMENT,
)

CURRENT_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_AMPERE,
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_CURRENT,
    state_class=STATE_CLASS_MEASUREMENT,
)

POWER_FACTOR_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="",
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_POWER_FACTOR,
    state_class=STATE_CLASS_MEASUREMENT,
)

PHASE_TABLES = {
    # VT (Voltage Transformers)
    "Jameco/112336": {"cal": 11.43, "phase": 0.6, "p60": [0.6], "base": 0.6},
    "JW-95001-NA": {"cal": 11.14, "phase": 0.74, "p60": [0.74], "base": 0.74},
    "Ideal 77DE-06-09(EU)": {"cal": 19.06, "p50": [1.46], "p60": [0.82], "base": 0.82},
    "Ideal 77DB-06-09(UK)": {"cal": 20.01, "p50": [1.05], "base": 1.05},
    "TDC DA-10-09-E6": {"cal": 10.79, "phase": 0.07, "p60": [0.07], "base": 0.07},
    "Jameco/2270815": {"cal": 10.72, "phase": 0.16, "p60": [0.16], "base": 0.16},
    "M9233": {"cal": 21.62, "phase": 1.81, "p50": [1.81], "base": 1.81},
    "DCSS AC910(Aus)": {"cal": 20.72, "p50": [2.17], "base": 2.17},
    "Powertech MP-3027(Aus)": {"cal": 23.18, "p50": [0.92], "base": 0.92},
    "Ideal 77DE-06-09-V(EU)": {"cal": 19.00, "p50": [0.82], "p60": [0.46], "base": 0.46},
    "Ideal 77DE-06-09-VI(EU)": {"cal": 18.98, "p50": [0.82], "p60": [0.36], "base": 0.36},
    "Ideal 77DA-10-09-MI(USA)": {"cal": 11.06, "p60": [0.10], "base": 0.10},
    "TDC DE-10-09(EU)": {"cal": 18.66, "p50": [0.16], "base": 0.16},
    "Jameco/100061": {"cal": 11.68, "p60": [3.00], "base": 3.00},
    "Jameco/102234": {"cal": 11.43, "p60": [1.80], "base": 1.80},
    "Jameco/2227444": {"cal": 11.51, "p60": [0.58], "base": 0.58},
    "CUI 41A-9-1000": {"cal": 10.10, "p60": [0.43], "base": 0.43},
    "AX-27750-H": {"cal": 20.56, "p60": [0.18], "base": 0.18},
    "TDC DA-10-09": {"cal": 11.20, "p60": [1.14], "base": 1.14},
    "generic120V": {"cal": 11.50, "phase": 2.0, "base": 2.0},
    "generic240V": {"cal": 20.00, "phase": 2.0, "base": 2.0},
    
    # CT (Current Transformers)
    "AcuCT-H040-50": {"turns": 1000, "p60": [1.1, 8, 1.0, 17, 0.9], "p50": [1.4, 3, 1.3, 6, 1.2, 10, 1.1, 17, 1.05, 40, 1.1], "base": 1.0},
    "AcuCT-H063-100": {"turns": 2000, "p60": [0.7, 15, 0.6, 45, 0.55], "p50": [0.8, 12, 0.7, 32, 0.6], "base": 0.6},
    "AcuCT-H100-200": {"turns": 4000, "p60": [0.3, 30, 0.25], "p50": [0.3, 50, 0.25], "base": 0.25},
    "AcuCT-H138-400": {"turns": 8000, "p60": [0.25, 10, 0.2, 30, 0.15], "p50": [0.3, 20, 0.2, 80, 0.15, 160, 0.10], "base": 0.15},
    "AcuCT-H138-600": {"turns": 12000, "p60": [0.1, 60, 0.05], "p50": [0.15, 20, 0.1, 100, 0.05], "base": 0.1},
    "SCT006-000": {"turns": 800, "base": 4.6},
    "SCT010-000": {"turns": 1000, "p60": [1.8, 2, 1.7, 8, 1.6, 15, 1.5, 35, 1.6, 45, 1.7], "p50": [2.1, 4, 2.0, 8, 1.9, 12, 1.8, 30, 1.9, 40, 2.1], "base": 1.75},
    "SCT013-000": {"turns": 2000, "p60": [2.7, 4, 2.6, 8, 2.5, 12, 2.4, 18, 2.3, 30, 2.2, 44, 2.1], "p50": [3.1, 4, 3.0, 8, 2.9, 12, 2.8, 17, 2.7, 26, 2.6, 38, 2.5], "base": 2.3},
    "SCT019-000": {"turns": 6072, "p60": [2.6, 3, 2.4, 5, 2.2, 10, 2.1, 18, 2.0, 35, 1.9], "p50": [3.2, 4, 3.0, 6, 2.8, 8, 2.7, 12, 2.6, 22, 2.5], "base": 1.8},
    "SCT023R": {"turns": 3780, "base": 0.8},
    "SCT027H": {"turns": 4000, "base": 2.0},
    "ECOL09": {"turns": 1000, "p60": [0.4, 20, 0.3], "p50": [0.5, 20, 0.4], "base": 0.10},
    "ECS1050": {"turns": 1000, "p60": [2.0, 1, 1.9, 2, 1.8, 3, 1.7, 6, 1.6, 12, 1.5, 21, 1.4], "p50": [2.2, 3, 2.1, 4, 2.0, 8, 1.9, 13, 1.8], "base": 1.6},
    "ECS16100": {"turns": 2000, "p60": [0.9, 7, 0.8, 30, 0.72], "p50": [1.2, 6, 1.1, 19, 1.0, 50, 0.95], "base": 0.6},
    "ECS24200": {"turns": 4000, "p60": [0.5, 20, 0.45, 55, 0.40], "p50": [0.8, 10, 0.7, 20, 0.6, 55, 0.55], "base": 0.2},
    "ECS25200-C2": {"turns": 4000, "p60": [1.1, 6, 1.0, 12, 0.9, 28, 0.8], "p50": [1.2, 8, 1.1, 18, 1.0, 36, 0.9], "base": 1.7},
    "ECS24300": {"turns": 6000, "p60": [0.5, 10, 0.4, 30, 0.3, 90, 0.25], "p50": [0.8, 10, 0.6, 30, 0.5, 90, 0.45], "base": 0.3},
    "ECS24400": {"turns": 8000, "p60": [0.5, 10, 0.4, 30, 0.3, 90, 0.25], "p50": [0.8, 10, 0.6, 30, 0.5, 90, 0.45], "base": 0.3},
    "ECS36400": {"turns": 8000, "p60": [0.1], "p50": [0.2, 10, 0.1], "base": 0.1},
    "ECS50600": {"turns": 12000, "p60": [0.4, 45, 0.35, 90, 0.3], "p50": [0.7, 20, 0.6, 35, 0.55], "base": 0.3},
    "ECSL55800": {"turns": 16000, "p60": [0.3, 12, 0.2, 40, 0.1], "p50": [0.6, 12, 0.3, 24, 0.2, 40, 0.15], "base": 0.1},
    "ECS80-800": {"turns": 16000, "base": 0.2},
    "ECS80-1200": {"turns": 24000, "base": 0.2},
    "ECS120-2000": {"turns": 40000, "base": 0.2},
    "CR3110-3000": {"turns": 3016, "base": 1.85},
    "WC-1": {"turns": 1000, "base": 1.6},
    "HWCT-004": {"turns": 1000, "p60": [0.4, 10, 0.3], "p50": [0.6, 10, 0.5], "base": 0.1},
    "DL-CT08CL2": {"turns": 1000, "p60": [0.2, 5, 0.1], "p50": [0.2, 5, 0.1, 15, 0.05], "base": 0.1},
    "AZ500": {"turns": 509, "p60": [3.0, 1.5, 3.5, 2.5, 3, 4, 2.5, 6.5, 2, 11, 1.5], "p50": [3.5, 2, 3.0, 3, 2.5, 5, 2.0, 10, 1.5, 20, 1.25], "base": 1.5},
    "AZ1000": {"turns": 1005, "p60": [1.5, 1, 1.25, 3, 1.0, 9, 0.75, 22, 0.5], "p50": [1.75, 2, 1.5, 3, 1.25, 7, 1.0, 17, 0.75, 33, 0.6], "base": 0.75},
    "Micro-40": {"turns": 1500, "p60": [3, 1, 2.5, 2, 2, 5, 1.5, 12, 1], "p50": [4, 2.5, 3.5, 3.5, 3, 5, 2.5, 7, 2, 12, 1.5, 35, 2], "base": 1.5},
    "ZDKCT10M": {"turns": 2007, "base": 1.5},
    "PZCT-02": {"turns": 1000, "base": 2.5},
    "SCT0750-000": {"turns": 7550, "base": 2.55},
    "SCT013-030": {"cal": 30, "base": 3.8},
    "SCT013-050": {"cal": 49.8, "base": 3.0},
}

def get_phase_table(model):
    if model and model in PHASE_TABLES:
        t = PHASE_TABLES[model]
        return t.get("p50", []), t.get("p60", []), t.get("base", 0.0)
    return [], [], 0.0


BASE_INPUT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=14),
        cv.Required(CONF_NAME): cv.string,
        cv.Required(CONF_TYPE): cv.one_of("VT", "CT"),
        cv.Optional(CONF_MODEL): cv.string,
        cv.Optional(CONF_CAL): cv.float_,
        cv.Optional(CONF_PHASE): cv.float_,
        cv.Optional(CONF_REVERSE, default=False): cv.boolean,
        cv.Optional(CONF_VPHASE, default=0.0): cv.float_,
        cv.Optional(CONF_DOUBLE, default=False): cv.boolean,
        cv.Optional(CONF_VOLTAGE_SENSOR): cv.Any(dict),
        cv.Optional(CONF_FREQUENCY_SENSOR): cv.Any(dict),
        cv.Optional(CONF_POWER_SENSOR): cv.Any(dict),
        cv.Optional(CONF_CURRENT_SENSOR): cv.Any(dict),
        cv.Optional(CONF_POWER_FACTOR): cv.Any(dict),
        cv.Optional("update_interval"): cv.invalid(
            "update_interval is a top-level option for the platform"
        ),
    }
)


def _validate_input(value):
    value = BASE_INPUT_SCHEMA(value)
    typ = value[CONF_TYPE]

    if typ == "VT":
        if CONF_VOLTAGE_SENSOR not in value:
            raise cv.Invalid("VT inputs require a 'voltage' sensor block.")
        if value.get(CONF_DOUBLE, False):
            raise cv.Invalid("'double' is only valid for CT inputs.")
        value[CONF_VOLTAGE_SENSOR] = VOLTAGE_SENSOR_SCHEMA(value[CONF_VOLTAGE_SENSOR])
        if CONF_FREQUENCY_SENSOR in value:
            value[CONF_FREQUENCY_SENSOR] = FREQUENCY_SENSOR_SCHEMA(
                value[CONF_FREQUENCY_SENSOR]
            )
    else:
        if CONF_POWER_SENSOR not in value:
            raise cv.Invalid("CT inputs require a 'power' sensor block.")
        if CONF_CURRENT_SENSOR not in value:
            raise cv.Invalid("CT inputs require a 'current' sensor block.")
        if CONF_FREQUENCY_SENSOR in value:
            raise cv.Invalid("'frequency' is only valid for VT inputs.")

        value[CONF_POWER_SENSOR] = POWER_SENSOR_SCHEMA(value[CONF_POWER_SENSOR])
        value[CONF_CURRENT_SENSOR] = CURRENT_SENSOR_SCHEMA(value[CONF_CURRENT_SENSOR])

        if CONF_POWER_FACTOR in value:
            value[CONF_POWER_FACTOR] = POWER_FACTOR_SENSOR_SCHEMA(
                value[CONF_POWER_FACTOR]
            )

    return value


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(IoTaWattComponent),
        cv.Required(CONF_INPUTS): cv.ensure_list(_validate_input),
        cv.Required(CONF_CLK_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_MOSI_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_MISO_PIN): pins.gpio_input_pin_schema,
        cv.Required(CONF_CS_PIN_0): pins.gpio_output_pin_schema,
        cv.Required(CONF_CS_PIN_1): pins.gpio_output_pin_schema,
    }
).extend(cv.polling_component_schema("5s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    clk = await cg.gpio_pin_expression(config[CONF_CLK_PIN])
    cg.add(var.set_clk_pin(clk))
    mosi = await cg.gpio_pin_expression(config[CONF_MOSI_PIN])
    cg.add(var.set_mosi_pin(mosi))
    miso = await cg.gpio_pin_expression(config[CONF_MISO_PIN])
    cg.add(var.set_miso_pin(miso))
    cs0 = await cg.gpio_pin_expression(config[CONF_CS_PIN_0])
    cg.add(var.set_cs_pin_0(cs0))
    cs1 = await cg.gpio_pin_expression(config[CONF_CS_PIN_1])
    cg.add(var.set_cs_pin_1(cs1))

    for input_conf in config[CONF_INPUTS]:
        ch = input_conf[CONF_CHANNEL]
        name = input_conf[CONF_NAME]
        typ = input_conf[CONF_TYPE]
        model = input_conf.get(CONF_MODEL)
        cal = input_conf.get(CONF_CAL)
        phase = input_conf.get(CONF_PHASE)
        reverse = input_conf[CONF_REVERSE]
        vphase = input_conf[CONF_VPHASE]
        double_input = input_conf[CONF_DOUBLE]
        frequency_s = None
        if CONF_FREQUENCY_SENSOR in input_conf:
            frequency_s = await sensor.new_sensor(input_conf[CONF_FREQUENCY_SENSOR])

        p50, p60, base = get_phase_table(model)
        
        # Handle default cal if not provided
        if cal is None:
            if model and model in PHASE_TABLES:
                m_data = PHASE_TABLES[model]
                if "cal" in m_data:
                    cal = m_data["cal"]
                elif "turns" in m_data:
                    cal = m_data["turns"] / 20.0 # 20 ohm burden resistor
                else:
                    cal = 1.0
            else:
                cal = 1.0

        # Handle default phase if not provided
        if phase is None:
            phase = base

        if typ == "VT":
            voltage_s = await sensor.new_sensor(input_conf[CONF_VOLTAGE_SENSOR])
            cg.add(
                var.add_input_vt(
                    ch,
                    name,
                    cal,
                    phase,
                    reverse,
                    voltage_s,
                    frequency_s,
                    p50,
                    p60,
                )
            )
            continue

        power_s = await sensor.new_sensor(input_conf[CONF_POWER_SENSOR])
        current_s = await sensor.new_sensor(input_conf[CONF_CURRENT_SENSOR])

        pf_s = None
        if CONF_POWER_FACTOR in input_conf:
            pf_s = await sensor.new_sensor(input_conf[CONF_POWER_FACTOR])

        cg.add(
            var.add_input_ct(
                ch,
                name,
                cal,
                phase,
                reverse,
                double_input,
                vphase,
                power_s,
                current_s,
                pf_s,
                p50,
                p60,
            )
        )
