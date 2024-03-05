#include <sjc_interface.hpp>
#include <iostream>

#include <stdexcept>
#include <string>
#include <cstring>
#include <chrono>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#include "helpers.h"
#include "network_configs.h"
}
#endif

typedef enum
{
    ACTUATOR_DISABLED,
    ACTUATOR_MAX_POS,
    ACTUATOR_MAX_NEG,
    ACTUATOR_STATE_SIZE
} eActuatorState_t;

std::string port;
std::string address;
std::string baudrate;

void sjc_initial_config(SjcInterface *SjcIface);

static eActuatorState_t xActuatorState = ACTUATOR_DISABLED;

int main(int argc, char *argv[])
{
    std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
    std::cout << "- SJC debug initialization" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    printf("\r\nProgram name is: %s\n", argv[0]);

    if (argc < 3)
    {
        printf("\r\n **** The standard call is: ./sjc_read_test port address baudrate****\r\n");
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

    port = argv[1];
    address = argv[2];
    baudrate = argv[3];

    std::cout << "\r\n\nModbus config:" << std::endl;
    std::cout << " - Port: " << port.c_str() << std::endl;
    std::cout << " - Address: " << std::stoi(address.c_str()) << std::endl;
    std::cout << " - Baudrate: " << std::stoi(baudrate.c_str()) << std::endl;

    SjcInterface Sjc(port.c_str(), std::stoi(address.c_str()), std::stoi(baudrate.c_str()));

    eSjcReturnType_t ret = Sjc.connect();
    uint32_t dur = 0;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    uint32_t last_time = 0;
    float last_dp = 0;

    uint32_t last_control_time = 0;

    if (ret == SJC_CONNECTED)
    {

        sjc_initial_config(&Sjc);

        do
        {
            static int cycle;

            xSjcRuntimeInfo_t RuntimeInfo;
            xSjcDeviceInfo_t DeviceInfo;
            xSjcNetworkConfig_t NetworkInfo;
            xSjcActuatorConfig_t ActuatorInfo;
            xSjcSensorsConfig_t SensorsInfo;
            xSjcControllerParameters_t ControllerInfo;

            end = std::chrono::steady_clock::now();
            dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            start = end;

            system("clear");

            std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
            std::cout << "- Cycle: " << cycle++ << std::endl;
            std::cout << "----------------------------------------------------------------------------------" << std::endl;

            if (SJC_RET_OK == Sjc.getRuntimeInfo(&RuntimeInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "System info: " << std::endl
                          << " * Uptime: " << RuntimeInfo.uptime;
                std::cout << " ms. - Input Voltage: " << std::to_string(RuntimeInfo.system_voltage) << " V - Error flags: " << RuntimeInfo.error_flags << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Controller:" << std::endl
                          << " * Mode: " << RuntimeInfo.controller_mode << " - Ref.: " << RuntimeInfo.controller_ref << " - Error: " << RuntimeInfo.controller_error << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Displacement sensors: " << std::endl
                          << " * Actuator: " << RuntimeInfo.actuator_displacement << " rd - Aux A: " << RuntimeInfo.aux_A_displacement << " rd - Aux B: " << RuntimeInfo.aux_B_displacement << " rd" << std::endl;
                float dt = (RuntimeInfo.uptime - last_time) / 1000.0f;
                last_time = RuntimeInfo.uptime;
                float ds = RuntimeInfo.actuator_displacement - last_dp;
                last_dp = RuntimeInfo.actuator_displacement;
                float vel = ds / dt;
                float vel_rpm = vel * (180 * 60) / (M_PI * 360);
                std::cout << " * Ds: " << std::to_string(ds) << " rad - Dt: " << std::to_string(dt) << " s" << std::endl;
                std::cout << " * Speed: " << (vel) << " rd/s (" << vel_rpm << " rpm)" << std::endl;

                std::cout << "----------------------------------" << std::endl;
                std::cout << "Actuator efforts: " << std::endl;
                std::cout << " * Voltage: " << std::to_string(RuntimeInfo.actuator_voltage) << " V";
                std::cout << " - Current: " << std::to_string(RuntimeInfo.actuator_current) << " A" << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "I/Os: " << std::endl;
                std::cout << " * SwA: " << RuntimeInfo.inputs.switchA;
                std::cout << " - SwB: " << RuntimeInfo.inputs.switchB;
                std::cout << " - Iso. InA: " << RuntimeInfo.inputs.iso_inputA;
                std::cout << " - Iso. InB: " << RuntimeInfo.inputs.iso_inputB;
                std::cout << " - OutA: " << RuntimeInfo.outputs.outputA;
                std::cout << " - OutB: " << RuntimeInfo.outputs.outputB << std::endl;
                std::cout << "----------------------------------" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << "Runtime info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            if ((RuntimeInfo.uptime - last_control_time) >= 100)
            {
                last_control_time = RuntimeInfo.uptime;
                xSjcControllerConfig_t config;
                config.controller_mode_update = SJC_OPEN_LOOP_CONTROL;
                static int sig = 1;
                switch (xActuatorState)
                {
                case ACTUATOR_MAX_POS:
                    xActuatorState = ACTUATOR_DISABLED;
                    sig *= -1;
                    config.controller_ref = 15 * sig;
                    break;

                default: // ACTUATOR_DISABLED
                    config.controller_ref = 0.0f;
                    xActuatorState = ACTUATOR_MAX_POS;
                    break;
                }

                Sjc.setControllerConfig(config);
            }

            if (SJC_RET_OK == Sjc.getActuatorParameters(&ActuatorInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Actuator parameters: " << std::endl;
                std::cout << " * State: ";
                if (ActuatorInfo.state > 0)
                {
                    std::cout << "Enabled!";
                }
                else
                {
                    std::cout << "Disabled!";
                }
                std::cout << " - Base voltage: " << ActuatorInfo.base_voltage << " V";
                std::cout << " - Current limit: " << ActuatorInfo.current_limit << " A";
                std::cout << " - Voltage limit: " << ActuatorInfo.voltage_limit << " V";
                std::cout << " - Speed limit: " << ActuatorInfo.speed_limit << " rpm" << std::endl;
                std::cout << "----------------------------------" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << "Actuator info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            if (SJC_RET_OK == Sjc.getSensorsParameters(&SensorsInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Sensors parameters: " << std::endl;
                std::cout << " * Actuator encoder: " << SensorsInfo.actuator_sensor_res_ppr << " ppr";
                std::cout << " - Aux A encoder: " << SensorsInfo.auxA_sensor_res_ppr << " ppr";
                std::cout << " - Aux B encoder: " << SensorsInfo.auxB_sensor_res_ppr << " ppr" << std::endl;
                std::cout << "----------------------------------" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << "Sensors info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            if (SJC_RET_OK == Sjc.getControllerParameters(&ControllerInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Controller parameters: " << std::endl;
                std::cout << " * Kp: " << std::to_string(ControllerInfo.kp_parameter);
                std::cout << " - Ki: " << std::to_string(ControllerInfo.ki_parameter);
                std::cout << " - Kd: " << std::to_string(ControllerInfo.kd_parameter);
                std::cout << " - Aux1: " << std::to_string(ControllerInfo.aux1_parameter);
                std::cout << " - Aux2: " << std::to_string(ControllerInfo.aux2_parameter);
                std::cout << " - Rate: " << std::to_string(ControllerInfo.rate_parameter) << std::endl;
                std::cout << "----------------------------------" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << "Sensors info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            std::cout << "----------------------------------" << std::endl;
            std::cout << " * Prev. Cycle duration: " << std::to_string(dur) << " ms." << std::endl;
            const auto end_cycle = std::chrono::steady_clock::now();
            uint32_t dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_cycle - start).count();
            int delta = (100 - dt) * 1e3;
            uint32_t wait = (delta < 0) ? 100e3 : delta;
            std::cout << " * Wait time: " << std::to_string(wait / 1000) << " ms" << std::endl;
            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;
            usleep(wait);

        } while (1);
    }

    Sjc.~SjcInterface();

    return 0;
}

void sjc_initial_config(SjcInterface *SjcIface)
{
    xSjcNetworkConfig_t xNetworkConfig;
    xSjcSensorsConfig_t xSensorsConfig;
    xSjcActuatorConfig_t xActuatorConfig;
    xSjcOutputsStates_t xOutputsStateConfig;
    xSjcControllerParameters_t xControllerParameters;

    xControllerParameters.kp_parameter = 1.0;
    xControllerParameters.ki_parameter = 0.0;
    xControllerParameters.kd_parameter = 1000.0;
    xControllerParameters.aux1_parameter = 10000.0;
    xControllerParameters.aux2_parameter = 30000.0;
    xControllerParameters.rate_parameter = 1000.0;

    xNetworkConfig.address = atoi(address.c_str());
    xNetworkConfig.parity = 'N';
    xNetworkConfig.baudrate = atoi(baudrate.c_str());

    xActuatorConfig.state = 1;
    xActuatorConfig.base_voltage = 24.0;
    xActuatorConfig.voltage_limit = 24.0;
    xActuatorConfig.current_limit = 10.0;
    xActuatorConfig.speed_limit = 7600.0;

    xSensorsConfig.actuator_sensor_res_ppr = 180;
    xSensorsConfig.auxA_sensor_res_ppr = 2048;
    xSensorsConfig.auxB_sensor_res_ppr = 2048;

    xOutputsStateConfig.outputA = false;
    xOutputsStateConfig.outputB = true;

    if (SJC_RET_NOK == SjcIface->setNetworkParameters(xNetworkConfig))
    {
        usleep(10e6);
        std::cout << " <|> Network parameters not set! \nPlease check the modbuss slave address..." << std::endl;
    }

    if (SJC_RET_NOK == SjcIface->setActuatorParameters(xActuatorConfig))
    {
        std::cout << " <|> Actuator parameters setted" << std::endl;
        usleep(500e3);
    }

    if (SJC_RET_NOK == SjcIface->setSensorsParameters(xSensorsConfig))
    {
        usleep(10e6);
        std::cout << " <|> Sensors parameters not set! \nPlease check the modbuss slave address..." << std::endl;
    }

    if (SJC_RET_NOK == SjcIface->setControllerParameters(xControllerParameters))
    {
        usleep(10e6);
        std::cout << " <|> Controller parameters not set! \nPlease check the modbuss slave address..." << std::endl;
    }

    if (SJC_RET_NOK == SjcIface->setDigitalOutputsStates(xOutputsStateConfig))
    {
        usleep(10e6);
        std::cout << " <|> Digital outputs stated not set! \nPlease check the modbuss slave address..." << std::endl;
    }
}