/*
* ODrive I2C communication library
* This file implements I2C communication with the ODrive.
*
*   - Implement the C function I2C_transaction to provide low level I2C access.
*   - Use read_property<PropertyId>() to read properties from the ODrive.
*   - Use write_property<PropertyId>() to modify properties on the ODrive.
*   - Use trigger<PropertyId>() to trigger a function (such as reboot or save_configuration)
*   - Use endpoint_type_t<PropertyId> to retrieve the underlying type
*     of a given property.
*   - Refer to PropertyId for a list of available properties.
*
* To regenerate the interface definitions, flash an ODrive with
* the new firmware, connect it to your PC via USB and then run
*   ../tools/odrivetool generate-code --output [path to odrive_endpoints.h]
python odrivetool generate-code --output odrive_endpoints.h
* This step can be done with any ODrive, it doesn't have to be the
* one that you'll be controlling over I2C.
*/

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QString>
#include <QFile>
#include <QMetaEnum>
#include <QDebug>

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <tuple>
#include <utility>
#include "odrive_endpoints.h"

#include <type_traits>
#include "fibre/posix_udp.hpp"
extern "C" {

/* @brief Send and receive data to/from an I2C slave
*
* This function carries out the following sequence:
* 1. generate a START condition
* 2. if the tx_buffer is not null:
*    a. send 7-bit slave address (with the LSB 0)
*    b. send all bytes in the tx_buffer
* 3. if both tx_buffer and rx_buffer are not null, generate a REPEATED START condition
* 4. if the rx_buffer is not null:
*    a. send 7-bit slave address (with the LSB 1)
*    b. read rx_length bytes into rx_buffer
* 5. send STOP condition
*
* @param slave_addr: 7-bit slave address (the MSB is ignored)
* @return true if all data was transmitted and received as requested by the caller, false otherwise
*/
//bool UDP_transaction(const uint8_t * tx_buffer, size_t tx_length, uint8_t * rx_buffer, size_t rx_length);

}

namespace odrive {
    template<typename T>
    using bit_width = std::integral_constant<unsigned int, CHAR_BIT * sizeof(T)>;

    template<typename T>
    using byte_width = std::integral_constant<unsigned int, (bit_width<T>::value + 7) / 8>;


    template<unsigned int IBitSize>
    struct unsigned_int_of_size;

    template<> struct unsigned_int_of_size<64> { typedef uint64_t type; };
    template<> struct unsigned_int_of_size<32> { typedef uint32_t type; };
    template<> struct unsigned_int_of_size<16> { typedef uint16_t type; };
    template<> struct unsigned_int_of_size<8> { typedef uint8_t type; };

    template<typename T>
    typename std::enable_if<std::is_integral<T>::value, T>::type
    read_le(const uint8_t buffer[byte_width<T>::value]) {
        T value = 0;
        for (size_t i = 0; i < byte_width<T>::value; ++i)
            value |= (static_cast<T>(buffer[i]) << (i << 3));
        return value;
    }

    template<typename T>
    typename std::enable_if<std::is_floating_point<T>::value, T>::type
    read_le(const uint8_t buffer[]) {
        using T_Int = typename unsigned_int_of_size<bit_width<T>::value>::type;
        T_Int value = read_le<T_Int>(buffer);
        return *reinterpret_cast<T*>(&value);
    }

    template<typename T>
    typename std::enable_if<std::is_integral<T>::value, void>::type
    write_le(uint8_t buffer[byte_width<T>::value], T value) {
        for (size_t i = 0; i < byte_width<T>::value; ++i)
            buffer[i] = (value >> (i << 3)) & 0xff;
    }

    template<typename T>
    typename std::enable_if<std::is_floating_point<T>::value, T>::type
    write_le(uint8_t buffer[byte_width<T>::value], T value) {
        using T_Int = typename unsigned_int_of_size<bit_width<T>::value>::type;
        write_le<T_Int>(buffer, *reinterpret_cast<T_Int*>(&value));
    }

    /* @brief Read from an endpoint on the ODrive.
    * To read from an axis specific endpoint use read_axis_property() instead.
    *
    * Usage example:
    *   float val;
    *   success = odrive::read_property<odrive::VBUS_VOLTAGE>(&val);
    *
    * @param num Selects the ODrive. For instance the value 4 selects
    * the ODrive that has [A2, A1, A0] connected to [VCC, GND, GND].
    * @return true if the transaction succeeded, false otherwise
    */
   /*
    template<int IPropertyId>
    bool read_property(endpoint_type_t<IPropertyId>* value, uint16_t address = IPropertyId) {
        uint8_t tx_buffer[8];
        address |= 0x8000;
        write_le<uint16_t>(tx_buffer, 0x8f);
        write_le<uint16_t>(tx_buffer + 2, address);
        write_le<uint16_t>(tx_buffer + 4, sizeof(endpoint_type_t<IPropertyId>));
        write_le<uint16_t>(tx_buffer + sizeof(tx_buffer) - 2, json_crc);
        uint8_t rx_buffer[2 + byte_width<endpoint_type_t<IPropertyId>>::value];
        if (!UDP_transaction(tx_buffer, sizeof(tx_buffer),
            rx_buffer, sizeof(rx_buffer)))
            return false;
        if (value)
            *value = read_le<endpoint_type_t<IPropertyId>>(rx_buffer + 2);
        return true;
    }
*/
    /* @brief Write to an endpoint on the ODrive.
    * To write to an axis specific endpoint use write_axis_property() instead.
    *
    * Usage example:
    *   success = odrive::write_property<odrive::TEST_PROPERTY>(42);
    *
    * @param num Selects the ODrive. For instance the value 4 selects
    * the ODrive that has [A2, A1, A0] connected to [VCC, GND, GND].
    * @return true if the transaction succeeded, false otherwise
    */
    /*
    template<int IPropertyId>
    bool write_property(endpoint_type_t<IPropertyId> value, uint16_t address = IPropertyId) {
        uint8_t tx_buffer[8 + byte_width<endpoint_type_t<IPropertyId>>::value];
        address |= 0x8000;
        write_le<uint16_t>(tx_buffer, 0x8f);
        write_le<uint16_t>(tx_buffer + 2, address);
        write_le<uint16_t>(tx_buffer + 4, 0);
        write_le<endpoint_type_t<IPropertyId>>(tx_buffer + 6, value);
        write_le<uint16_t>(tx_buffer + sizeof(tx_buffer) - 2, json_crc);
        uint8_t rx_buffer[2];
        return UDP_transaction(tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer));
    }
*/
    template<int IPropertyId>
    bool read_property(fibre_udp_socket socket, endpoint_type_t<IPropertyId>* value, uint16_t address = IPropertyId) {
        using T_Int = typename unsigned_int_of_size<bit_width<endpoint_type_t<IPropertyId>>::value>::type;
        constexpr size_t size = sizeof(endpoint_type_t<IPropertyId>);
        T_Int buf;
        uint8_t tx_buffer[8];
        address |= 0x8000;
        write_le<uint16_t>(tx_buffer, 0x8f);
        write_le<uint16_t>(tx_buffer + 2, address);
        write_le<uint16_t>(tx_buffer + 4, size);
        write_le<uint16_t>(tx_buffer + sizeof(tx_buffer) - 2, json_crc);
        uint8_t rx_buffer[2 + byte_width<endpoint_type_t<IPropertyId>>::value];
        if (!socket.UDP_transaction(tx_buffer, sizeof(tx_buffer),
                             rx_buffer, sizeof(rx_buffer)))
            return false;
        if (value)
        {
            buf = read_le<T_Int>(rx_buffer + 2);
            memcpy((char *)value, (char *)&buf, sizeof(buf));
        }
        return true;
    }

    template<int IPropertyId>
    bool write_property(fibre_udp_socket socket, endpoint_type_t<IPropertyId> value, uint16_t address = IPropertyId) {
        using T_Int = typename unsigned_int_of_size<bit_width<endpoint_type_t<IPropertyId>>::value>::type;
        T_Int buf;
        uint8_t tx_buffer[8 + byte_width<endpoint_type_t<IPropertyId>>::value];
        memcpy((char *)&buf, (char *)&value, sizeof(value));
        address |= 0x8000;
        write_le<uint16_t>(tx_buffer, 0x8f);
        write_le<uint16_t>(tx_buffer + 2, address);
        write_le<uint16_t>(tx_buffer + 4, 0);
        write_le<T_Int>(tx_buffer + 6, buf);
        write_le<uint16_t>(tx_buffer + sizeof(tx_buffer) - 2, json_crc);
        uint8_t rx_buffer[2];
        return socket.UDP_transaction(tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer));
    }

    template<int IPropertyId>
    bool read_axis_property(fibre_udp_socket socket, uint8_t axis, endpoint_type_t<IPropertyId>* value) {
        return read_property<IPropertyId>(socket, value, IPropertyId + axis * per_axis_offset);
    }

    template<int IPropertyId>
    bool write_axis_property(fibre_udp_socket socket, uint8_t axis, endpoint_type_t<IPropertyId> value) {
        return write_property<IPropertyId>(socket, value, IPropertyId + axis * per_axis_offset);
    }
    /* @brief Trigger an parameter-less function on the ODrive
    *
    * Usage example:
    *   success = odrive::trigger<odrive::SAVE_CONFIGURATION>();
    *
    * @param num Selects the ODrive. For instance the value 4 selects
    * the ODrive that has [A2, A1, A0] connected to [VCC, GND, GND].
    * @return true if the transaction succeeded, false otherwise
    */

    template<int IPropertyId,
             typename = typename std::enable_if<std::is_void<endpoint_type_t<IPropertyId>>::value>::type>
    bool trigger(fibre_udp_socket socket, uint16_t address = IPropertyId) {
        uint8_t tx_buffer[8];
        address |= 0x8000;
        write_le<uint16_t>(tx_buffer, 0x8f);
        write_le<uint16_t>(tx_buffer + 2, address);
        write_le<uint16_t>(tx_buffer + 4, 0);
        write_le<uint16_t>(tx_buffer + sizeof(tx_buffer) - 2, json_crc);
        uint8_t rx_buffer[2];
        return socket.UDP_transaction(tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer));
    }

    template<int IPropertyId, typename Args, std::size_t N>
    struct traverse
    {
        static bool write(fibre_udp_socket socket, Args t, uint16_t address = IPropertyId)
        {
            bool success = false;
            //qDebug() << std::get<N-1>(t);
            success = write_property<IPropertyId + N>(socket, std::get<N-1>(t), address+N);
            if (!traverse<IPropertyId, Args, N-1>::write(socket, t, address))
                return false;
            return true;
        }
        static bool read(fibre_udp_socket socket, Args t, uint16_t address = IPropertyId)
        {
            bool success = false;
            //qDebug() << std::get<N-1>(t);
            success = read_property<IPropertyId + N>(socket, std::get<N-1>(t), address+N);
            if (!traverse<IPropertyId, Args, N-1>::read(socket, t, address))
                return false;
            return true;
        }
    };

    template<int IPropertyId, typename Args>
    struct traverse<IPropertyId, Args, 0>
    {
        static bool write(fibre_udp_socket socket, Args t, uint16_t address = IPropertyId)
        {
            return true;
        }
        static bool read(fibre_udp_socket socket, Args t, uint16_t address = IPropertyId)
        {
            return true;
        }
    };

    template<int IPropertyId, typename... INPUT, typename... OUTPUT>
    bool func_call(fibre_udp_socket socket, std::tuple<INPUT...> inargs, std::tuple<OUTPUT...> outargs, uint16_t address = IPropertyId) {
        bool success = false;
        success = traverse<IPropertyId, decltype(inargs), sizeof...(INPUT)>::write(socket, inargs, address);
        if(success)
        {
            success = trigger<IPropertyId>(socket, address);
        }
        if(success)
        {
            success = traverse<IPropertyId, decltype(outargs), sizeof...(OUTPUT)>::read(socket, outargs, address+sizeof...(INPUT));
        }
        return success;
    }

    template<int IPropertyId, typename... INPUT, typename... OUTPUT>
    bool axis_func_call(fibre_udp_socket socket, uint8_t axis, std::tuple<INPUT...> inargs, std::tuple<OUTPUT...> outargs) {
        bool success = false;
        success = func_call<IPropertyId>(socket, inargs, outargs, IPropertyId + axis * per_axis_offset);
        return success;
    }

    template<int IPropertyId>
    bool set_config(fibre_udp_socket socket, QJsonObject *object, QMetaEnum *metaenum, uint16_t address = IPropertyId) {
        endpoint_type_t<IPropertyId> value;
        QString name;
        QStringList namelist;
        QJsonValue jsonvalue;

        namelist = QString(metaenum->valueToKey(IPropertyId)).split("CONFIG__");
        name = namelist[1].toLower();

        if(name.isEmpty())
            return false;
        jsonvalue = object->take(name);

        if(jsonvalue.isDouble())
            value = jsonvalue.toVariant().toDouble();
        else if(jsonvalue.isBool())
            value = jsonvalue.toVariant().toBool();
        else
            value = jsonvalue.toVariant().toUInt();

        return write_property<IPropertyId>(socket, value, address);
    }

    template<int IPropertyId>
    bool get_config(fibre_udp_socket socket, QJsonObject *object, QMetaEnum *metaenum, uint16_t address = IPropertyId) {
        endpoint_type_t<IPropertyId> value;
        QString name;
        QStringList namelist;

        bool success = read_property<IPropertyId>(socket, &value, address);
        if(success)
        {
            namelist = QString(metaenum->valueToKey(IPropertyId)).split("CONFIG__");
            name = namelist[1].toLower();
            if(name.isEmpty())
                return false;
            qDebug() << name << value;
            object->insert(name, QJsonValue(value));
        }

        return success;
    }

    template<int IPropertyId>
    bool set_axis_config(fibre_udp_socket socket, uint8_t axis, QJsonObject *object, QMetaEnum *metaenum) {
        return set_config<IPropertyId>(socket, object, metaenum, IPropertyId + axis * per_axis_offset);
    }

    template<int IPropertyId>
    bool get_axis_config(fibre_udp_socket socket, uint8_t axis, QJsonObject *object, QMetaEnum *metaenum) {
        return get_config<IPropertyId>(socket, object, metaenum, IPropertyId + axis * per_axis_offset);
    }
    bool restore_config(fibre_udp_socket socket, QFile &file){
        bool success;
        //QFile file("E:/code/test_fibre/test_fibre_client/config_file.json");
        if(!file.open(QIODevice::ReadWrite)){
            return false;
        }
        QMetaEnum metaenum = QMetaEnum::fromType<fibre::fibre_id>();
        QJsonParseError result;
        QJsonDocument jsondoc = QJsonDocument::fromJson(file.readAll(),&result);
        file.close();

        QJsonObject configobj;
        QJsonObject rootobj;
        QJsonObject axisobj[axis_count];
        QJsonObject axisconfig[axis_count];
        QJsonObject motorobj[axis_count];
        QJsonObject motorconfig[axis_count];
        QJsonObject encoderobj[axis_count];
        QJsonObject encoderconfig[axis_count];
        QJsonObject controllerobj[axis_count];
        QJsonObject controllerconfig[axis_count];
        QJsonObject sensorless_estimatorobj[axis_count];
        QJsonObject sensorless_estimatorconfig[axis_count];
        QJsonObject traptrajobj[axis_count];
        QJsonObject traptrajconfig[axis_count];

        if(!jsondoc.isNull() && result.error == QJsonParseError::NoError)
        {
            rootobj = jsondoc.object();
            if(rootobj.contains("config"))
            {
                configobj = rootobj.value("config").toObject();
                success = success ? set_config<fibre::CONFIG__BRAKE_RESISTANCE>(socket, &configobj, &metaenum):false;
                success = set_config<fibre::CONFIG__ENABLE_UART>(socket, &configobj, &metaenum);
                success = set_config<fibre::CONFIG__ENABLE_I2C_INSTEAD_OF_CAN>(socket, &configobj, &metaenum);
                success = set_config<fibre::CONFIG__ENABLE_ASCII_PROTOCOL_ON_USB>(socket, &configobj, &metaenum);
                success = set_config<fibre::CONFIG__DC_BUS_UNDERVOLTAGE_TRIP_LEVEL>(socket, &configobj, &metaenum);
                success = set_config<fibre::CONFIG__DC_BUS_OVERVOLTAGE_TRIP_LEVEL>(socket, &configobj, &metaenum);
            }
            for(uint8_t i = 0;i < axis_count;i++)
            {
                axisobj[i] = rootobj.value(QString("axis%1").arg(i)).toObject();

                axisconfig[i] = axisobj[i].value("config").toObject();
                success = set_axis_config<fibre::AXIS__CONFIG__STARTUP_MOTOR_CALIBRATION>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__STARTUP_ENCODER_INDEX_SEARCH>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__STARTUP_ENCODER_OFFSET_CALIBRATION>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__STARTUP_CLOSED_LOOP_CONTROL>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__STARTUP_SENSORLESS_CONTROL>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__ENABLE_STEP_DIR>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__COUNTS_PER_STEP>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__WATCHDOG_TIMEOUT>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__STEP_GPIO_PIN>(socket, i, &axisconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONFIG__DIR_GPIO_PIN>(socket, i, &axisconfig[i], &metaenum);

                motorobj[i] = axisobj[i].value("motor").toObject();
                motorconfig[i] = motorobj[i].value("config").toObject();
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__PRE_CALIBRATED>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__POLE_PAIRS>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__CALIBRATION_CURRENT>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__RESISTANCE_CALIB_MAX_VOLTAGE>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__PHASE_INDUCTANCE>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__PHASE_RESISTANCE>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__DIRECTION>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__MOTOR_TYPE>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__CURRENT_LIM>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__CURRENT_LIM_TOLERANCE>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_LOWER>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_UPPER>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__REQUESTED_CURRENT_RANGE>(socket, i, &motorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__MOTOR__CONFIG__CURRENT_CONTROL_BANDWIDTH>(socket, i, &motorconfig[i], &metaenum);

                encoderobj[i] = axisobj[i].value("encoder").toObject();
                encoderconfig[i] = encoderobj[i].value("config").toObject();
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__MODE>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__USE_INDEX>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__FIND_IDX_ON_LOCKIN_ONLY>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__PRE_CALIBRATED>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__ZERO_COUNT_ON_FIND_IDX>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__CPR>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__OFFSET>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__OFFSET_FLOAT>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__ENABLE_PHASE_INTERPOLATION>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__BANDWIDTH>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__CALIB_RANGE>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__CALIB_SCAN_DISTANCE>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__CALIB_SCAN_OMEGA>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__IDX_SEARCH_UNIDIRECTIONAL>(socket, i, &encoderconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__ENCODER__CONFIG__IGNORE_ILLEGAL_HALL_STATE>(socket, i, &encoderconfig[i], &metaenum);

                controllerobj[i] = axisobj[i].value("controller").toObject();
                controllerconfig[i] = controllerobj[i].value("config").toObject();
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__CONTROL_MODE>(socket, i, &controllerconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__POS_GAIN>(socket, i, &controllerconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_GAIN>(socket, i, &controllerconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_INTEGRATOR_GAIN>(socket, i, &controllerconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_LIMIT>(socket, i, &controllerconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_LIMIT_TOLERANCE>(socket, i, &controllerconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE>(socket, i, &controllerconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__CONTROLLER__CONFIG__SETPOINTS_IN_CPR>(socket, i, &controllerconfig[i], &metaenum);

                sensorless_estimatorobj[i] = axisobj[i].value("sensorless_estimator").toObject();
                sensorless_estimatorconfig[i] = sensorless_estimatorobj[i].value("config").toObject();
                success = set_axis_config<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__OBSERVER_GAIN>(socket, i, &sensorless_estimatorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__PLL_BANDWIDTH>(socket, i, &sensorless_estimatorconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__PM_FLUX_LINKAGE>(socket, i, &sensorless_estimatorconfig[i], &metaenum);

                traptrajobj[i] = axisobj[i].value("traptraj").toObject();
                traptrajconfig[i] = traptrajobj[i].value("config").toObject();
                success = set_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__VEL_LIMIT>(socket, i, &traptrajconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__ACCEL_LIMIT>(socket, i, &traptrajconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__DECEL_LIMIT>(socket, i, &traptrajconfig[i], &metaenum);
                success = set_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__A_PER_CSS>(socket, i, &traptrajconfig[i], &metaenum);
            }
        }
        return success;
    }

    bool save_config(fibre_udp_socket socket, QFile &file){
        bool success;
        //QFile file("E:/code/test_fibre/test_fibre_client/config_file.json");
        if(!file.open(QIODevice::ReadWrite)){
            return false;
        }
        QMetaEnum metaenum = QMetaEnum::fromType<fibre::fibre_id>();

        QJsonDocument jsondoc;
        QJsonObject configobj;
        QJsonObject rootobj;
        QJsonObject axisobj[axis_count];
        QJsonObject axisconfig[axis_count];
        QJsonObject motorobj[axis_count];
        QJsonObject motorconfig[axis_count];
        QJsonObject encoderobj[axis_count];
        QJsonObject encoderconfig[axis_count];
        QJsonObject controllerobj[axis_count];
        QJsonObject controllerconfig[axis_count];
        QJsonObject sensorless_estimatorobj[axis_count];
        QJsonObject sensorless_estimatorconfig[axis_count];
        QJsonObject traptrajobj[axis_count];
        QJsonObject traptrajconfig[axis_count];

        success = get_config<fibre::CONFIG__BRAKE_RESISTANCE>(socket, &configobj, &metaenum);
        success = get_config<fibre::CONFIG__ENABLE_UART>(socket, &configobj, &metaenum);
        success = get_config<fibre::CONFIG__ENABLE_I2C_INSTEAD_OF_CAN>(socket, &configobj, &metaenum);
        success = get_config<fibre::CONFIG__ENABLE_ASCII_PROTOCOL_ON_USB>(socket, &configobj, &metaenum);
        success = get_config<fibre::CONFIG__DC_BUS_UNDERVOLTAGE_TRIP_LEVEL>(socket, &configobj, &metaenum);
        success = get_config<fibre::CONFIG__DC_BUS_OVERVOLTAGE_TRIP_LEVEL>(socket, &configobj, &metaenum);
        rootobj.insert("config", configobj);

        for(uint8_t i = 0;i < axis_count;i++)
        {
            success = get_axis_config<fibre::AXIS__CONFIG__STARTUP_MOTOR_CALIBRATION>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__STARTUP_ENCODER_INDEX_SEARCH>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__STARTUP_ENCODER_OFFSET_CALIBRATION>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__STARTUP_CLOSED_LOOP_CONTROL>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__STARTUP_SENSORLESS_CONTROL>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__ENABLE_STEP_DIR>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__COUNTS_PER_STEP>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__WATCHDOG_TIMEOUT>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__STEP_GPIO_PIN>(socket, i, &axisconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONFIG__DIR_GPIO_PIN>(socket, i, &axisconfig[i], &metaenum);
            axisobj[i].insert("config", axisconfig[i]);

            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__PRE_CALIBRATED>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__POLE_PAIRS>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__CALIBRATION_CURRENT>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__RESISTANCE_CALIB_MAX_VOLTAGE>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__PHASE_INDUCTANCE>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__PHASE_RESISTANCE>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__DIRECTION>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__MOTOR_TYPE>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__CURRENT_LIM>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__CURRENT_LIM_TOLERANCE>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_LOWER>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__INVERTER_TEMP_LIMIT_UPPER>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__REQUESTED_CURRENT_RANGE>(socket, i, &motorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__MOTOR__CONFIG__CURRENT_CONTROL_BANDWIDTH>(socket, i, &motorconfig[i], &metaenum);
            motorobj[i].insert("config", motorconfig[i]);
            axisobj[i].insert("motor", motorobj[i]);

            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__MODE>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__USE_INDEX>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__FIND_IDX_ON_LOCKIN_ONLY>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__PRE_CALIBRATED>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__ZERO_COUNT_ON_FIND_IDX>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__CPR>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__OFFSET>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__OFFSET_FLOAT>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__ENABLE_PHASE_INTERPOLATION>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__BANDWIDTH>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__CALIB_RANGE>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__CALIB_SCAN_DISTANCE>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__CALIB_SCAN_OMEGA>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__IDX_SEARCH_UNIDIRECTIONAL>(socket, i, &encoderconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__ENCODER__CONFIG__IGNORE_ILLEGAL_HALL_STATE>(socket, i, &encoderconfig[i], &metaenum);
            encoderobj[i].insert("config", encoderconfig[i]);
            axisobj[i].insert("encoder", encoderobj[i]);

            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__CONTROL_MODE>(socket, i, &controllerconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__POS_GAIN>(socket, i, &controllerconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_GAIN>(socket, i, &controllerconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_INTEGRATOR_GAIN>(socket, i, &controllerconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_LIMIT>(socket, i, &controllerconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_LIMIT_TOLERANCE>(socket, i, &controllerconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE>(socket, i, &controllerconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__CONTROLLER__CONFIG__SETPOINTS_IN_CPR>(socket, i, &controllerconfig[i], &metaenum);
            controllerobj[i].insert("config", controllerconfig[i]);
            axisobj[i].insert("controller", controllerobj[i]);

            success = get_axis_config<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__OBSERVER_GAIN>(socket, i, &sensorless_estimatorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__PLL_BANDWIDTH>(socket, i, &sensorless_estimatorconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__SENSORLESS_ESTIMATOR__CONFIG__PM_FLUX_LINKAGE>(socket, i, &sensorless_estimatorconfig[i], &metaenum);
            sensorless_estimatorobj[i].insert("config", sensorless_estimatorconfig[i]);
            axisobj[i].insert("sensorless_estimator", sensorless_estimatorobj[i]);

            success = get_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__VEL_LIMIT>(socket, i, &traptrajconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__ACCEL_LIMIT>(socket, i, &traptrajconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__DECEL_LIMIT>(socket, i, &traptrajconfig[i], &metaenum);
            success = get_axis_config<fibre::AXIS__TRAP_TRAJ__CONFIG__A_PER_CSS>(socket, i, &traptrajconfig[i], &metaenum);
            traptrajobj[i].insert("config", traptrajconfig[i]);
            axisobj[i].insert("traptraj", traptrajobj[i]);

            rootobj.insert(QString("axis%1").arg(i), axisobj[i]);
        }

        jsondoc.setObject(rootobj);

        file.write(jsondoc.toJson());
        file.close();

        return success;
    }
}
