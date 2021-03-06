/*
* This file was autogenerated using the "odrivetool generate-code" feature.
*
* The file matches a specific firmware version. If you add/remove/rename any
* properties exposed by the ODrive, this file needs to be regenerated, otherwise
* the ODrive will ignore all commands.
*/

#ifndef __ODRIVE_ENDPOINTS_HPP
#define __ODRIVE_ENDPOINTS_HPP
#include <stdint.h>
#include <QObject>


typedef struct {
    uint16_t json_crc;
    uint16_t endpoint_id;
} endpoint_ref_t;

class fibre:public QObject {
    Q_OBJECT
public:
    enum fibre_id{ 
        FW_VERSION_MAJOR = 1,
        FW_VERSION_MINOR = 2,
        FW_VERSION_REVISION = 3,
        FW_VERSION_UNRELEASED = 4,
        USER_CONFIG_LOADED = 5,
        SYSTEM_STATS__UPTIME = 6,
        SYSTEM_STATS__MIN_HEAP_SPACE = 7,
        SYSTEM_STATS__MIN_STACK_SPACE_AXIS0 = 8,
        SYSTEM_STATS__MIN_STACK_SPACE_AXIS1 = 9,
        SYSTEM_STATS__MIN_STACK_SPACE_COMMS = 10,
        SYSTEM_STATS__MIN_STACK_SPACE_USB = 11,
        SYSTEM_STATS__MIN_STACK_SPACE_UART = 12,
        SYSTEM_STATS__MIN_STACK_SPACE_USB_IRQ = 13,
        SYSTEM_STATS__MIN_STACK_SPACE_STARTUP = 14,
        CONFIG__BRAKE_RESISTANCE = 15,
        CONFIG__ENABLE_UART = 16,
        CONFIG__ENABLE_I2C_INSTEAD_OF_CAN = 17,
        CONFIG__ENABLE_ASCII_PROTOCOL_ON_USB = 18,
        CONFIG__DC_BUS_UNDERVOLTAGE_TRIP_LEVEL = 19,
        CONFIG__DC_BUS_OVERVOLTAGE_TRIP_LEVEL = 20,
        CONFIG__GPIO1_PWM_MAPPING__ENDPOINT = 21,
        CONFIG__GPIO1_PWM_MAPPING__MIN = 22,
        CONFIG__GPIO1_PWM_MAPPING__MAX = 23,
        CONFIG__GPIO2_ANALOG_MAPPING__ENDPOINT = 24,
        CONFIG__GPIO2_ANALOG_MAPPING__MIN = 25,
        CONFIG__GPIO2_ANALOG_MAPPING__MAX = 26,
        TEST_FUNCTION = 323,
        TEST_FUNCTION__DELTA = 324,
        TEST_FUNCTION__RESULT = 325,
        GET_OSCILLOSCOPE_VAL = 326,
        GET_OSCILLOSCOPE_VAL__INDEX = 327,
        GET_OSCILLOSCOPE_VAL__RESULT = 328,
        GET_ADC_VOLTAGE = 329,
        GET_ADC_VOLTAGE__GPIO = 330,
        GET_ADC_VOLTAGE__RESULT = 331,
        SAVE_CONFIGURATION = 332,
        ERASE_CONFIGURATION = 333,
        REBOOT = 334,
        ENTER_DFU_MODE = 335,

        // Per-Axis endpoints (to be used with read_axis_property and write_axis_property)
        AXIS__ERROR = 27,
        AXIS__STEP_DIR_ACTIVE = 28,
        AXIS__CURRENT_STATE = 29,
        AXIS__REQUESTED_STATE = 30,
        AXIS__LOOP_COUNTER = 31,
        AXIS__LOCKIN_STATE = 32,
        AXIS__CONFIG__STARTUP_MOTOR_CALIBRATION = 33,
        AXIS__CONFIG__STARTUP_ENCODER_INDEX_SEARCH = 34,
        AXIS__CONFIG__STARTUP_ENCODER_OFFSET_CALIBRATION = 35,
        AXIS__CONFIG__STARTUP_CLOSED_LOOP_CONTROL = 36,
        AXIS__CONFIG__STARTUP_SENSORLESS_CONTROL = 37,
        AXIS__CONFIG__ENABLE_STEP_DIR = 38,
        AXIS__CONFIG__COUNTS_PER_STEP = 39,
        AXIS__CONFIG__WATCHDOG_TIMEOUT = 40,
        AXIS__CONFIG__STEP_GPIO_PIN = 41,
        AXIS__CONFIG__DIR_GPIO_PIN = 42,
        AXIS__CONFIG__CALIBRATION_LOCKIN__CURRENT = 43,
        AXIS__CONFIG__CALIBRATION_LOCKIN__RAMP_TIME = 44,
        AXIS__CONFIG__CALIBRATION_LOCKIN__RAMP_DISTANCE = 45,
        AXIS__CONFIG__CALIBRATION_LOCKIN__ACCEL = 46,
        AXIS__CONFIG__CALIBRATION_LOCKIN__VEL = 47,
        AXIS__CONFIG__SENSORLESS_RAMP__CURRENT = 48,
        AXIS__CONFIG__SENSORLESS_RAMP__RAMP_TIME = 49,
        AXIS__CONFIG__SENSORLESS_RAMP__RAMP_DISTANCE = 50,
        AXIS__CONFIG__SENSORLESS_RAMP__ACCEL = 51,
        AXIS__CONFIG__SENSORLESS_RAMP__VEL = 52,
        AXIS__CONFIG__SENSORLESS_RAMP__FINISH_DISTANCE = 53,
        AXIS__CONFIG__SENSORLESS_RAMP__FINISH_ON_VEL = 54,
        AXIS__CONFIG__SENSORLESS_RAMP__FINISH_ON_DISTANCE = 55,
        AXIS__CONFIG__SENSORLESS_RAMP__FINISH_ON_ENC_IDX = 56,
        AXIS__CONFIG__GENERAL_LOCKIN__CURRENT = 57,
        AXIS__CONFIG__GENERAL_LOCKIN__RAMP_TIME = 58,
        AXIS__CONFIG__GENERAL_LOCKIN__RAMP_DISTANCE = 59,
        AXIS__CONFIG__GENERAL_LOCKIN__ACCEL = 60,
        AXIS__CONFIG__GENERAL_LOCKIN__VEL = 61,
        AXIS__CONFIG__GENERAL_LOCKIN__FINISH_DISTANCE = 62,
        AXIS__CONFIG__GENERAL_LOCKIN__FINISH_ON_VEL = 63,
        AXIS__CONFIG__GENERAL_LOCKIN__FINISH_ON_DISTANCE = 64,
        AXIS__CONFIG__GENERAL_LOCKIN__FINISH_ON_ENC_IDX = 65,
        AXIS__MOTOR__ERROR = 66,
        AXIS__MOTOR__ARMED_STATE = 67,
        AXIS__MOTOR__IS_CALIBRATED = 68,
        AXIS__MOTOR__CURRENT_MEAS_PHB = 69,
        AXIS__MOTOR__CURRENT_MEAS_PHC = 70,
        AXIS__MOTOR__DC_CALIB_PHB = 71,
        AXIS__MOTOR__DC_CALIB_PHC = 72,
        AXIS__MOTOR__PHASE_CURRENT_REV_GAIN = 73,
        AXIS__MOTOR__THERMAL_CURRENT_LIM = 74,
        AXIS__MOTOR__CURRENT_CONTROL__P_GAIN = 75,
        AXIS__MOTOR__CURRENT_CONTROL__I_GAIN = 76,
        AXIS__MOTOR__CURRENT_CONTROL__V_CURRENT_CONTROL_INTEGRAL_D = 77,
        AXIS__MOTOR__CURRENT_CONTROL__V_CURRENT_CONTROL_INTEGRAL_Q = 78,
        AXIS__MOTOR__CURRENT_CONTROL__IBUS = 79,
        AXIS__MOTOR__CURRENT_CONTROL__FINAL_V_ALPHA = 80,
        AXIS__MOTOR__CURRENT_CONTROL__FINAL_V_BETA = 81,
        AXIS__MOTOR__CURRENT_CONTROL__IQ_SETPOINT = 82,
        AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED = 83,
        AXIS__MOTOR__CURRENT_CONTROL__ID_MEASURED = 84,
        AXIS__MOTOR__CURRENT_CONTROL__I_MEASURED_REPORT_FILTER_K = 85,
        AXIS__MOTOR__CURRENT_CONTROL__MAX_ALLOWED_CURRENT = 86,
        AXIS__MOTOR__CURRENT_CONTROL__OVERCURRENT_TRIP_LEVEL = 87,
        AXIS__MOTOR__CONFIG__PRE_CALIBRATED = 88,
        AXIS__MOTOR__CONFIG__POLE_PAIRS = 89,
        AXIS__MOTOR__CONFIG__CALIBRATION_CURRENT = 90,
        AXIS__MOTOR__CONFIG__RESISTANCE_CALIB_MAX_VOLTAGE = 91,
        AXIS__MOTOR__CONFIG__PHASE_INDUCTANCE = 92,
        AXIS__MOTOR__CONFIG__PHASE_RESISTANCE = 93,
        AXIS__MOTOR__CONFIG__DIRECTION = 94,
        AXIS__MOTOR__CONFIG__MOTOR_TYPE = 95,
        AXIS__MOTOR__CONFIG__CURRENT_LIM = 96,
        AXIS__MOTOR__CONFIG__CURRENT_LIM_TOLERANCE = 97,
        AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_LOWER = 98,
        AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_UPPER = 99,
        AXIS__MOTOR__CONFIG__REQUESTED_CURRENT_RANGE = 100,
        AXIS__MOTOR__CONFIG__CURRENT_CONTROL_BANDWIDTH = 101,
        AXIS__MOTOR__GET_INVERTER_TEMP = 102,
        AXIS__MOTOR__GET_INVERTER_TEMP__RESULT = 103,
        AXIS__CONTROLLER__ERROR = 104,
        AXIS__CONTROLLER__POS_SETPOINT = 105,
        AXIS__CONTROLLER__VEL_SETPOINT = 106,
        AXIS__CONTROLLER__VEL_INTEGRATOR_CURRENT = 107,
        AXIS__CONTROLLER__CURRENT_SETPOINT = 108,
        AXIS__CONTROLLER__VEL_RAMP_TARGET = 109,
        AXIS__CONTROLLER__VEL_RAMP_ENABLE = 110,
        AXIS__CONTROLLER__CONFIG__CONTROL_MODE = 111,
        AXIS__CONTROLLER__CONFIG__POS_GAIN = 112,
        AXIS__CONTROLLER__CONFIG__VEL_GAIN = 113,
        AXIS__CONTROLLER__CONFIG__VEL_INTEGRATOR_GAIN = 114,
        AXIS__CONTROLLER__CONFIG__VEL_LIMIT = 115,
        AXIS__CONTROLLER__CONFIG__VEL_LIMIT_TOLERANCE = 116,
        AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE = 117,
        AXIS__CONTROLLER__CONFIG__SETPOINTS_IN_CPR = 118,
        AXIS__CONTROLLER__SET_POS_SETPOINT = 119,
        AXIS__CONTROLLER__SET_POS_SETPOINT__POS_SETPOINT = 120,
        AXIS__CONTROLLER__SET_POS_SETPOINT__VEL_FEED_FORWARD = 121,
        AXIS__CONTROLLER__SET_POS_SETPOINT__CURRENT_FEED_FORWARD = 122,
        AXIS__CONTROLLER__SET_VEL_SETPOINT = 123,
        AXIS__CONTROLLER__SET_VEL_SETPOINT__VEL_SETPOINT = 124,
        AXIS__CONTROLLER__SET_VEL_SETPOINT__CURRENT_FEED_FORWARD = 125,
        AXIS__CONTROLLER__SET_CURRENT_SETPOINT = 126,
        AXIS__CONTROLLER__SET_CURRENT_SETPOINT__CURRENT_SETPOINT = 127,
        AXIS__CONTROLLER__MOVE_TO_POS = 128,
        AXIS__CONTROLLER__MOVE_TO_POS__POS_SETPOINT = 129,
        AXIS__CONTROLLER__MOVE_INCREMENTAL = 130,
        AXIS__CONTROLLER__MOVE_INCREMENTAL__DISPLACEMENT = 131,
        AXIS__CONTROLLER__MOVE_INCREMENTAL__FROM_GOAL_POINT = 132,
        AXIS__CONTROLLER__START_ANTICOGGING_CALIBRATION = 133,
        AXIS__ENCODER__ERROR = 134,
        AXIS__ENCODER__IS_READY = 135,
        AXIS__ENCODER__INDEX_FOUND = 136,
        AXIS__ENCODER__SHADOW_COUNT = 137,
        AXIS__ENCODER__COUNT_IN_CPR = 138,
        AXIS__ENCODER__INTERPOLATION = 139,
        AXIS__ENCODER__PHASE = 140,
        AXIS__ENCODER__POS_ESTIMATE = 141,
        AXIS__ENCODER__POS_CPR = 142,
        AXIS__ENCODER__HALL_STATE = 143,
        AXIS__ENCODER__VEL_ESTIMATE = 144,
        AXIS__ENCODER__CALIB_SCAN_RESPONSE = 145,
        AXIS__ENCODER__CONFIG__MODE = 146,
        AXIS__ENCODER__CONFIG__USE_INDEX = 147,
        AXIS__ENCODER__CONFIG__FIND_IDX_ON_LOCKIN_ONLY = 148,
        AXIS__ENCODER__CONFIG__PRE_CALIBRATED = 149,
        AXIS__ENCODER__CONFIG__ZERO_COUNT_ON_FIND_IDX = 150,
        AXIS__ENCODER__CONFIG__CPR = 151,
        AXIS__ENCODER__CONFIG__OFFSET = 152,
        AXIS__ENCODER__CONFIG__OFFSET_FLOAT = 153,
        AXIS__ENCODER__CONFIG__ENABLE_PHASE_INTERPOLATION = 154,
        AXIS__ENCODER__CONFIG__BANDWIDTH = 155,
        AXIS__ENCODER__CONFIG__CALIB_RANGE = 156,
        AXIS__ENCODER__CONFIG__CALIB_SCAN_DISTANCE = 157,
        AXIS__ENCODER__CONFIG__CALIB_SCAN_OMEGA = 158,
        AXIS__ENCODER__CONFIG__IDX_SEARCH_UNIDIRECTIONAL = 159,
        AXIS__ENCODER__CONFIG__IGNORE_ILLEGAL_HALL_STATE = 160,
        AXIS__ENCODER__SET_LINEAR_COUNT = 161,
        AXIS__ENCODER__SET_LINEAR_COUNT__COUNT = 162,
        AXIS__SENSORLESS_ESTIMATOR__ERROR = 163,
        AXIS__SENSORLESS_ESTIMATOR__PHASE = 164,
        AXIS__SENSORLESS_ESTIMATOR__PLL_POS = 165,
        AXIS__SENSORLESS_ESTIMATOR__VEL_ESTIMATE = 166,
        AXIS__SENSORLESS_ESTIMATOR__CONFIG__OBSERVER_GAIN = 167,
        AXIS__SENSORLESS_ESTIMATOR__CONFIG__PLL_BANDWIDTH = 168,
        AXIS__SENSORLESS_ESTIMATOR__CONFIG__PM_FLUX_LINKAGE = 169,
        AXIS__TRAP_TRAJ__CONFIG__VEL_LIMIT = 170,
        AXIS__TRAP_TRAJ__CONFIG__ACCEL_LIMIT = 171,
        AXIS__TRAP_TRAJ__CONFIG__DECEL_LIMIT = 172,
        AXIS__TRAP_TRAJ__CONFIG__A_PER_CSS = 173,
        AXIS__WATCHDOG_FEED = 174,
    };
    Q_ENUM(fibre_id)
};

namespace odrive {

static constexpr const uint8_t axis_count = 2;    
    
static constexpr const uint16_t json_crc = 0x4e48;

static constexpr const uint16_t per_axis_offset = 148;

template<int I>
struct endpoint_type;

template<> struct endpoint_type<fibre::FW_VERSION_MAJOR> { typedef uint8_t type; };
template<> struct endpoint_type<fibre::FW_VERSION_MINOR> { typedef uint8_t type; };
template<> struct endpoint_type<fibre::FW_VERSION_REVISION> { typedef uint8_t type; };
template<> struct endpoint_type<fibre::FW_VERSION_UNRELEASED> { typedef uint8_t type; };
template<> struct endpoint_type<fibre::USER_CONFIG_LOADED> { typedef bool type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__UPTIME> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_HEAP_SPACE> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_STACK_SPACE_AXIS0> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_STACK_SPACE_AXIS1> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_STACK_SPACE_COMMS> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_STACK_SPACE_USB> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_STACK_SPACE_UART> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_STACK_SPACE_USB_IRQ> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::SYSTEM_STATS__MIN_STACK_SPACE_STARTUP> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::CONFIG__BRAKE_RESISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::CONFIG__ENABLE_UART> { typedef bool type; };
template<> struct endpoint_type<fibre::CONFIG__ENABLE_I2C_INSTEAD_OF_CAN> { typedef bool type; };
template<> struct endpoint_type<fibre::CONFIG__ENABLE_ASCII_PROTOCOL_ON_USB> { typedef bool type; };
template<> struct endpoint_type<fibre::CONFIG__DC_BUS_UNDERVOLTAGE_TRIP_LEVEL> { typedef float type; };
template<> struct endpoint_type<fibre::CONFIG__DC_BUS_OVERVOLTAGE_TRIP_LEVEL> { typedef float type; };
template<> struct endpoint_type<fibre::CONFIG__GPIO1_PWM_MAPPING__ENDPOINT> { typedef endpoint_ref_t type; };
template<> struct endpoint_type<fibre::CONFIG__GPIO1_PWM_MAPPING__MIN> { typedef float type; };
template<> struct endpoint_type<fibre::CONFIG__GPIO1_PWM_MAPPING__MAX> { typedef float type; };
template<> struct endpoint_type<fibre::CONFIG__GPIO2_ANALOG_MAPPING__ENDPOINT> { typedef endpoint_ref_t type; };
template<> struct endpoint_type<fibre::CONFIG__GPIO2_ANALOG_MAPPING__MIN> { typedef float type; };
template<> struct endpoint_type<fibre::CONFIG__GPIO2_ANALOG_MAPPING__MAX> { typedef float type; };
template<> struct endpoint_type<fibre::TEST_FUNCTION> { typedef void type; };
template<> struct endpoint_type<fibre::TEST_FUNCTION__DELTA> { typedef int32_t type; };
template<> struct endpoint_type<fibre::TEST_FUNCTION__RESULT> { typedef int32_t type; };
template<> struct endpoint_type<fibre::GET_OSCILLOSCOPE_VAL> { typedef void type; };
template<> struct endpoint_type<fibre::GET_OSCILLOSCOPE_VAL__INDEX> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::GET_OSCILLOSCOPE_VAL__RESULT> { typedef float type; };
template<> struct endpoint_type<fibre::GET_ADC_VOLTAGE> { typedef void type; };
template<> struct endpoint_type<fibre::GET_ADC_VOLTAGE__GPIO> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::GET_ADC_VOLTAGE__RESULT> { typedef float type; };
template<> struct endpoint_type<fibre::SAVE_CONFIGURATION> { typedef void type; };
template<> struct endpoint_type<fibre::ERASE_CONFIGURATION> { typedef void type; };
template<> struct endpoint_type<fibre::REBOOT> { typedef void type; };
template<> struct endpoint_type<fibre::ENTER_DFU_MODE> { typedef void type; };


// Per-axis endpoints
template<> struct endpoint_type<fibre::AXIS__ERROR> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__STEP_DIR_ACTIVE> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CURRENT_STATE> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__REQUESTED_STATE> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__LOOP_COUNTER> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__LOCKIN_STATE> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__STARTUP_MOTOR_CALIBRATION> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__STARTUP_ENCODER_INDEX_SEARCH> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__STARTUP_ENCODER_OFFSET_CALIBRATION> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__STARTUP_CLOSED_LOOP_CONTROL> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__STARTUP_SENSORLESS_CONTROL> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__ENABLE_STEP_DIR> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__COUNTS_PER_STEP> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__WATCHDOG_TIMEOUT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__STEP_GPIO_PIN> { typedef uint16_t type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__DIR_GPIO_PIN> { typedef uint16_t type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__CALIBRATION_LOCKIN__CURRENT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__CALIBRATION_LOCKIN__RAMP_TIME> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__CALIBRATION_LOCKIN__RAMP_DISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__CALIBRATION_LOCKIN__ACCEL> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__CALIBRATION_LOCKIN__VEL> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__CURRENT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__RAMP_TIME> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__RAMP_DISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__ACCEL> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__VEL> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__FINISH_DISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__FINISH_ON_VEL> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__FINISH_ON_DISTANCE> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__SENSORLESS_RAMP__FINISH_ON_ENC_IDX> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__CURRENT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__RAMP_TIME> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__RAMP_DISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__ACCEL> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__VEL> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__FINISH_DISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__FINISH_ON_VEL> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__FINISH_ON_DISTANCE> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONFIG__GENERAL_LOCKIN__FINISH_ON_ENC_IDX> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__ERROR> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__ARMED_STATE> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__IS_CALIBRATED> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_MEAS_PHB> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_MEAS_PHC> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__DC_CALIB_PHB> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__DC_CALIB_PHC> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__PHASE_CURRENT_REV_GAIN> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__THERMAL_CURRENT_LIM> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__P_GAIN> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__I_GAIN> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__V_CURRENT_CONTROL_INTEGRAL_D> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__V_CURRENT_CONTROL_INTEGRAL_Q> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__IBUS> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__FINAL_V_ALPHA> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__FINAL_V_BETA> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__IQ_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__ID_MEASURED> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__I_MEASURED_REPORT_FILTER_K> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__MAX_ALLOWED_CURRENT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CURRENT_CONTROL__OVERCURRENT_TRIP_LEVEL> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__PRE_CALIBRATED> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__POLE_PAIRS> { typedef int32_t type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__CALIBRATION_CURRENT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__RESISTANCE_CALIB_MAX_VOLTAGE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__PHASE_INDUCTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__PHASE_RESISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__DIRECTION> { typedef int32_t type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__MOTOR_TYPE> { typedef uint16_t type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__CURRENT_LIM> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__CURRENT_LIM_TOLERANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_LOWER> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_UPPER> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__REQUESTED_CURRENT_RANGE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__CONFIG__CURRENT_CONTROL_BANDWIDTH> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__GET_INVERTER_TEMP> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__MOTOR__GET_INVERTER_TEMP__RESULT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__ERROR> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__POS_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__VEL_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__VEL_INTEGRATOR_CURRENT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CURRENT_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__VEL_RAMP_TARGET> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__VEL_RAMP_ENABLE> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__CONTROL_MODE> { typedef uint16_t type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__POS_GAIN> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__VEL_GAIN> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__VEL_INTEGRATOR_GAIN> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__VEL_LIMIT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__VEL_LIMIT_TOLERANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__CONFIG__SETPOINTS_IN_CPR> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_POS_SETPOINT> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_POS_SETPOINT__POS_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_POS_SETPOINT__VEL_FEED_FORWARD> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_POS_SETPOINT__CURRENT_FEED_FORWARD> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_VEL_SETPOINT> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_VEL_SETPOINT__VEL_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_VEL_SETPOINT__CURRENT_FEED_FORWARD> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_CURRENT_SETPOINT> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__SET_CURRENT_SETPOINT__CURRENT_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__MOVE_TO_POS> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__MOVE_TO_POS__POS_SETPOINT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__MOVE_INCREMENTAL> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__MOVE_INCREMENTAL__DISPLACEMENT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__MOVE_INCREMENTAL__FROM_GOAL_POINT> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__CONTROLLER__START_ANTICOGGING_CALIBRATION> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__ERROR> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__IS_READY> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__INDEX_FOUND> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__SHADOW_COUNT> { typedef int32_t type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__COUNT_IN_CPR> { typedef int32_t type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__INTERPOLATION> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__PHASE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__POS_ESTIMATE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__POS_CPR> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__HALL_STATE> { typedef uint8_t type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__VEL_ESTIMATE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CALIB_SCAN_RESPONSE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__MODE> { typedef uint16_t type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__USE_INDEX> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__FIND_IDX_ON_LOCKIN_ONLY> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__PRE_CALIBRATED> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__ZERO_COUNT_ON_FIND_IDX> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__CPR> { typedef int32_t type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__OFFSET> { typedef int32_t type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__OFFSET_FLOAT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__ENABLE_PHASE_INTERPOLATION> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__BANDWIDTH> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__CALIB_RANGE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__CALIB_SCAN_DISTANCE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__CALIB_SCAN_OMEGA> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__IDX_SEARCH_UNIDIRECTIONAL> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__CONFIG__IGNORE_ILLEGAL_HALL_STATE> { typedef bool type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__SET_LINEAR_COUNT> { typedef void type; };
template<> struct endpoint_type<fibre::AXIS__ENCODER__SET_LINEAR_COUNT__COUNT> { typedef int32_t type; };
template<> struct endpoint_type<fibre::AXIS__SENSORLESS_ESTIMATOR__ERROR> { typedef uint32_t type; };
template<> struct endpoint_type<fibre::AXIS__SENSORLESS_ESTIMATOR__PHASE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__SENSORLESS_ESTIMATOR__PLL_POS> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__SENSORLESS_ESTIMATOR__VEL_ESTIMATE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__OBSERVER_GAIN> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__PLL_BANDWIDTH> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__PM_FLUX_LINKAGE> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__TRAP_TRAJ__CONFIG__VEL_LIMIT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__TRAP_TRAJ__CONFIG__ACCEL_LIMIT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__TRAP_TRAJ__CONFIG__DECEL_LIMIT> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__TRAP_TRAJ__CONFIG__A_PER_CSS> { typedef float type; };
template<> struct endpoint_type<fibre::AXIS__WATCHDOG_FEED> { typedef void type; };


template<int I>
using endpoint_type_t = typename endpoint_type<I>::type;

}

#endif // __ODRIVE_ENDPOINTS_HPP