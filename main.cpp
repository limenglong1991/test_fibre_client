#include <QCoreApplication>
#include <QDebug>
#include "fibre/posix_udp.hpp"
#include "odrive.h"
#include "odrive_endpoints.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    fibre_udp_socket socket;
    socket.client_udp_init(9910);

//    odrive::endpoint_type_t<odrive::MAPPING> val;
//    bool success = odrive::read_property<odrive::MAPPING>(&val);
//    qDebug() << val.json_crc << val.endpoint_id;
//    val.json_crc = odrive::json_crc+10;
//    val.endpoint_id = odrive::FW_VERSION_MAJOR;
//    success = odrive::write_property<odrive::MAPPING>(val);

    //odrive::endpoint_type_t<fibre::AXIS__CONTROLLER__CURRENT_SETPOINT> setpoint;
    //setpoint = 5.0;
    //bool success = odrive::write_axis_property<fibre::AXIS__CONTROLLER__CURRENT_SETPOINT>(socket, 1, setpoint);

    odrive::endpoint_type_t<fibre::AXIS__CONTROLLER__CURRENT_SETPOINT> val;
    bool success = odrive::read_axis_property<fibre::AXIS__CONTROLLER__CURRENT_SETPOINT>(socket, 1, &val);
    qDebug() << val;

    while(true)
    {
        odrive::endpoint_type_t<fibre::AXIS__ENCODER__VEL_ESTIMATE> val;
        success = odrive::read_axis_property<fibre::AXIS__ENCODER__VEL_ESTIMATE>(socket, 1, &val);
        qDebug() << val;
    }
//    success = odrive::write_property<1>(val);

//    bool success = odrive::trigger<odrive::TEST_FUN1>(std::make_tuple(), std::make_tuple());
//    qDebug() << success;

    //QFile file("E:/code/test_fibre/test_fibre_client/config_file.json");
    //odrive::restore_config(socket, file);
    //odrive::save_config(socket, file);
//    float a = 1.5;
//    float b = 2.5;
//    float c = 3.5;
//    bool success = odrive::axis_func_call<fibre::AXIS__CONTROLLER__SET_POS_SETPOINT>(socket, 1, std::make_tuple(a, b, c), std::make_tuple());
//    qDebug() << success;

    socket.client_udp_close();
    return app.exec();
}
