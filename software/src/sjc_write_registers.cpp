#include <sjc_interface.hpp>
#include <iostream>

#include <stdexcept>
#include <string>
#include <cstring>
#include <chrono>

#ifdef __cplusplus
extern "C"
{
#include "helpers.h"
#include "network_configs.h"
}
#endif

int main(int argc, char *argv[])
{
    std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
    std::cout << "- SJC debug initialization" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    printf("\r\nProgram name is: %s\n", argv[0]);

    if (argc < 3)
    {
        printf("\r\n **** The standard call is: ./sjc_write_test port address baudrate****\r\n");
        printf("\r\n ****  - port: COMX or /dev/ttyUSBX ****\r\n");
        printf("\r\n ****  - address: 1 to 246 ****\r\n");
        printf("\r\n ****  - baudrate: 115200 to 921600 ****\r\n");

        if (argc == 1)
            printf("\nNo Extra Command Line Argument Passed "
                   "Other Than Program Name");

        if (argc >= 2)
        {
            printf("\nNumber Of Arguments Passed: %d", argc);
            printf("\n----Following Are The Command Line "
                   "Arguments Passed----");

            for (int i = 0; i < argc; i++)
            {
                printf("\nargv[%d]: %s", i, argv[i]);
            }
        }

        return -1;
    }

    std::string port = argv[1];
    std::string address = argv[2];
    std::string baudrate = argv[3];

    std::cout << "\r\n\nModbus config:" << std::endl;
    std::cout << " - Port: " << port.c_str() << std::endl;
    std::cout << " - Address: " << std::stoi(address.c_str()) << std::endl;
    std::cout << " - Baudrate: " << std::stoi(baudrate.c_str()) << std::endl;

    SjcInterface Sjc(port.c_str(), std::stoi(address.c_str()), std::stoi(baudrate.c_str()));

    eSjcReturnType_t ret = Sjc.connect();

    if (ret == SJC_CONNECTED)
    {
        // do
        {
            static int cycle;

            xSjcNetworkConfig_t xNetworkConfig;
            xSjcSensorsConfig_t xSensorsConfig;
            xSjcActuatorConfig_t xActuatorConfig;
            xSjcOutputsStates_t xOutputsStateConfig;
            xSjcControllerParameters_t xControllerParameters;

            system("clear");
            // const auto start = std::chrono::steady_clock::now();

            std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
            std::cout << "- Cycle: " << cycle++ << std::endl;
            std::cout << "----------------------------------------------------------------------------------" << std::endl;

            xNetworkConfig.address = atoi(address.c_str());
            xNetworkConfig.parity = 'N';
            xNetworkConfig.baudrate = atoi(baudrate.c_str());

            std::cout << "Network config: " << std::endl;
            std::cout << " * Address: " << std::to_string(xNetworkConfig.address);
            std::cout << " - Parity: " << xNetworkConfig.parity;
            std::cout << " - Baudrate: " << xNetworkConfig.baudrate << " bps" << std::endl;
            if (SJC_RET_OK == Sjc.setNetworkParameters(xNetworkConfig))
            {
                std::cout << " <|> Network parameters setted" << std::endl;
                usleep(500e3);
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Network parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            xActuatorConfig.state = 1;
            xActuatorConfig.base_voltage = 24.0;
            xActuatorConfig.voltage_limit = 24.0;
            xActuatorConfig.current_limit = 10.0;
            xActuatorConfig.speed_limit = 7600.0;

            std::cout << "Actuator config: " << std::endl;
            std::cout << " * State: " << ((xActuatorConfig.state > 0) ? "En" : "Dis");
            std::cout << " - Base Voltage: " << xActuatorConfig.base_voltage << " V";
            std::cout << " - Voltage limit: " << xActuatorConfig.voltage_limit << " V";
            std::cout << " - Current limit: " << xActuatorConfig.current_limit << " A";
            std::cout << " - Speed limit: " << xActuatorConfig.speed_limit << " rpm" << std::endl;

            if (SJC_RET_OK == Sjc.setActuatorParameters(xActuatorConfig))
            {
                std::cout << " <|> Actuator parameters setted" << std::endl;
                usleep(500e3);
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Actuator parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            xSensorsConfig.actuator_sensor_resolution = 2048 * 4;
            xSensorsConfig.aux_sensor_resolution_a = 2048 * 4;
            xSensorsConfig.aux_sensor_resolution_b = 2048 * 4;

            std::cout << "Sensors config: " << std::endl;
            std::cout << " * Actuator resolution: " << xSensorsConfig.actuator_sensor_resolution << " ppr ";
            std::cout << " - Aux A resolution: " << xSensorsConfig.aux_sensor_resolution_a << " ppr ";
            std::cout << " - Aux B resolution: " << xSensorsConfig.aux_sensor_resolution_a << " ppr " << std::endl;

            if (SJC_RET_OK == Sjc.setSensorsParameters(xSensorsConfig))
            {
                std::cout << " <|> Sensors parameters setted" << std::endl;
                usleep(500e3);
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Sensors parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            // xControllerParameters.Kp = 0.0050077;
            // xControllerParameters.Ki = 0.23959;
            xControllerParameters.Kp = 0.0113;
            xControllerParameters.Ki = 0.5;
            xControllerParameters.Kd = 0.0;
            xControllerParameters.Kt = 10000.0;
            xControllerParameters.Ka = 30000.0;
            xControllerParameters.update_rate = 1000.0;

            std::cout << "Controller parameters: " << std::endl;
            std::cout << " * Kp: " << xControllerParameters.Kp;
            std::cout << " - Ki: " << xControllerParameters.Ki;
            std::cout << " - Kd: " << xControllerParameters.Kd;
            std::cout << " - Aux1: " << xControllerParameters.Kt;
            std::cout << " - Aux2: " << xControllerParameters.Ka;
            std::cout << " - Rate: " << xControllerParameters.update_rate << std::endl;

            if (SJC_RET_OK == Sjc.setControllerParameters(xControllerParameters))
            {
                std::cout << " <|> Controller parameters setted" << std::endl;
                usleep(500e3);
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Controller parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            float controller_ref = 0.0;
            eSjcControlMode_t controller_mode_update = SJC_OPEN_LOOP_CONTROL;

            std::cout << "Controller config: " << std::endl;
            std::cout << " * Mode: ";
            switch (controller_mode_update)
            {
            case SJC_OPEN_LOOP_CONTROL:
                std::cout << "Open loop";
                break;
            case SJC_PRBS_EXCITATION:
                std::cout << "PRBS Excitation";
                break;
            case SJC_SPEED_CONTROL:
                std::cout << "Speed control";
                break;
            case SJC_CURRENT_CONTROL:
                std::cout << "Torque control";
                break;

            default: // SJC_CONTROL_DISABLED
                std::cout << "Control disabled";
                break;
            }
            std::cout << " - Ref: " << controller_ref << std::endl;

            if ((SJC_RET_OK == Sjc.setControllerOperatingMode(controller_mode_update)) && (SJC_RET_OK == Sjc.setControllerReference(controller_ref)))
            {
                std::cout << " <|> Controller configs setted" << std::endl;
                usleep(500e3);
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Controller configs not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            xOutputsStateConfig.output_a = false;
            xOutputsStateConfig.output_b = true;

            std::cout << "Digital outputs: " << std::endl;
            std::cout << " * OutA: " << std::to_string(xOutputsStateConfig.output_a);
            std::cout << " - OutB: " << std::to_string(xOutputsStateConfig.output_b);

            if (SJC_RET_OK == Sjc.setDigitalOutputsStates(xOutputsStateConfig))
            {
                std::cout << " <|> Digital outputs stated setted" << std::endl;
                usleep(500e3);
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Digital outputs stated not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            // const auto end = std::chrono::steady_clock::now();
            // const std::chrono::duration<double> diff = end - start;
            // std::cout << "Com. time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms." << std::endl;
            // std::cout << "----------------------------------" << std::endl;
            // usleep(1e6);
            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;
        }
    }

    // while (1);

    Sjc.~SjcInterface();

    return 0;
}