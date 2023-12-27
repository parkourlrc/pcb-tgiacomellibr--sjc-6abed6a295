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
            xSjcControllerConfig_t xControllerConfig;

            system("clear");
            const auto start = std::chrono::steady_clock::now();

            std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
            std::cout << "- Cycle: " << cycle++ << std::endl;
            std::cout << "----------------------------------------------------------------------------------" << std::endl;
            
            xNetworkConfig.address = NETWORK_BASE_ADRESS;
            xNetworkConfig.parity = 'N';
            xNetworkConfig.baudrate = NETWORK_BAUD_RATE;

            std::cout << "Network config: " << std::endl;
            std::cout << " * Address: " << std::to_string(xNetworkConfig.address);
            std::cout << " - Parity: " << xNetworkConfig.parity;            
            std::cout << " - Baudrate: " << xNetworkConfig.baudrate << " bps" << std::endl;            
            if (SJC_RET_OK == Sjc.setNetworkParameters(xNetworkConfig))
            {
                std::cout << " <|> Network parameters setted" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Network parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            xActuatorConfig.state = 1;
            xActuatorConfig.base_voltage = 24.0;
            xActuatorConfig.voltage_limit = 23.5;
            xActuatorConfig.current_limit = 9.8;
            xActuatorConfig.speed_limit = 1.0;

            std::cout << "Controller config: " << std::endl;
            std::cout << " * State: " << ( (xActuatorConfig.state > 0) ? "En" : "Dis" );
            std::cout << " - Base Voltage: " << xActuatorConfig.base_voltage << " V";
            std::cout << " - Voltage limit: " << xActuatorConfig.voltage_limit << " V";
            std::cout << " - Current limit: " << xActuatorConfig.current_limit << " A";
            std::cout << " - Speed limit: " << xActuatorConfig.speed_limit << " rd/s" << std::endl;
            
            if (SJC_RET_OK == Sjc.setActuatorParameters(xActuatorConfig))
            {
                std::cout << " <|> Actuator parameters setted" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Actuator parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            xSensorsConfig.actuator_sensor_res_ppr = 2048;
            xSensorsConfig.auxA_sensor_res_ppr= 2048;
            xSensorsConfig.auxB_sensor_res_ppr= 2048;

            std::cout << "Sensors config: " << std::endl;
            std::cout << " * Actuator resolution: " << xSensorsConfig.actuator_sensor_res_ppr*4 << " ppr (Quad.)";
            std::cout << " - Aux A resolution: " << xSensorsConfig.auxA_sensor_res_ppr*4 << " ppr (Quad.)";
            std::cout << " - Aux B resolution: " << xSensorsConfig.auxA_sensor_res_ppr*4 << " ppr (Quad.)" << std::endl;
            
            if (SJC_RET_OK == Sjc.setSensorsParameters(xSensorsConfig))
            {
                std::cout << " <|> Sensors parameters setted" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Sensors parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            xControllerConfig.kp_parameter = 1.0;
            xControllerConfig.ki_parameter = 0.0;
            xControllerConfig.kd_parameter = 1000.0;
            xControllerConfig.aux1_parameter = 10000.0;
            xControllerConfig.aux2_parameter = 30000.0;
            xControllerConfig.rate_parameter = 50000.0;

            std::cout << "Controller config: " << std::endl;
            std::cout << " * Kp: "   << xControllerConfig.kp_parameter;
            std::cout << " - Ki: "   << xControllerConfig.ki_parameter;
            std::cout << " - Kd: "   << xControllerConfig.kd_parameter;
            std::cout << " - Aux1: " << xControllerConfig.aux1_parameter;
            std::cout << " - Aux2: " << xControllerConfig.aux2_parameter;
            std::cout << " - Rate: " << xControllerConfig.rate_parameter << std::endl;
            
            if (SJC_RET_OK == Sjc.setControllerParameters(xControllerConfig))
            {
                std::cout << " <|> Sensors parameters setted" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Sensors parameters not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            xOutputsStateConfig.outputA = false;
            xOutputsStateConfig.outputB = true;

            std::cout << "Digital outputs: " << std::endl;
            std::cout << " * OutA: "   << std::to_string(xOutputsStateConfig.outputA);
            std::cout << " - OutB: "   << std::to_string(xOutputsStateConfig.outputB);
            
            if (SJC_RET_OK == Sjc.setDigitalOutputsStates(xOutputsStateConfig))
            {
                std::cout << " <|> Digital outputs stated setted" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Digital outputs stated not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;

            const auto end = std::chrono::steady_clock::now();
            const std::chrono::duration<double> diff = end - start;
            std::cout << "Com. time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms." << std::endl;
            std::cout << "----------------------------------" << std::endl;
            usleep(1e6);
            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;
        }
    }

    // while (1);

    Sjc.~SjcInterface();

    return 0;
}
