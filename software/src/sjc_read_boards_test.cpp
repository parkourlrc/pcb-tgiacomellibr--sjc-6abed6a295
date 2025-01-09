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
}
#endif

typedef enum
{
    CRANE_X_JOINT = 0,
    CRANE_Y_JOINT,
    CRANE_L_JOINT,
    CRANE_JOINTS_SIZE
} eCraneJoint_t;

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

    std::string port = argv[1];
    std::string address = argv[2];
    std::string baudrate = argv[3];

    std::cout << "\r\n\nModbus config:" << std::endl;
    std::cout << " - Port: " << port.c_str() << std::endl;
    std::cout << " - Address: " << std::stoi(address.c_str()) << std::endl;
    std::cout << " - Baudrate: " << std::stoi(baudrate.c_str()) << std::endl;

    SjcInterface Sjc(port.c_str(), std::stoi(address.c_str()), std::stoi(baudrate.c_str()));

    eSjcReturnType_t ret = Sjc.connect();
    uint32_t dur = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    // auto tic = std::chrono::high_resolution_clock::now();

    uint32_t last_time = 0;
    float last_dp = 0;

    if (ret == SJC_CONNECTED)
    {
        do
        {
            static int cycle;

            xSjcRuntimeInfo_t RuntimeInfo;
            xSjcControllerConfig_t xControllerConfig;

            end = std::chrono::high_resolution_clock::now();
            dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            start = end;

            system("clear");

            std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
            std::cout << "- Cycle: " << cycle++ << std::endl;
            std::cout << "----------------------------------------------------------------------------------" << std::endl;
            auto tic = std::chrono::high_resolution_clock::now();

            for (int idx = 0; idx < CRANE_JOINTS_SIZE; idx++)
            {

                if (SJC_RET_OK == Sjc.getRuntimeInfo(&RuntimeInfo))
                {
                    printf("\rJoint[%d] ------------------------", idx);
                    // std::cout << "----------------------------------" << std::endl;
                    std::cout << "System info: " << std::endl
                              << " * Uptime: " << RuntimeInfo.uptime;
                    std::cout << " ms. - Input Voltage: " << std::to_string(RuntimeInfo.system_voltage) << " V - Error flags: " << RuntimeInfo.error_flags << std::endl;
                    std::cout << "----------------------------------" << std::endl;
                    // std::cout << "Controller:" << std::endl
                    //           << " * Mode: " << RuntimeInfo.controller_mode << " - Ref.: " << RuntimeInfo.controller_ref << " - Error: " << RuntimeInfo.controller_error << std::endl;
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
                    // std::cout << "I/Os: " << std::endl;
                    // std::cout << " * SwA: " << RuntimeInfo.inputs.switchA;
                    // std::cout << " - SwB: " << RuntimeInfo.inputs.switchB;
                    // std::cout << " - Iso. InA: " << RuntimeInfo.inputs.iso_inputA;
                    // std::cout << " - Iso. InB: " << RuntimeInfo.inputs.iso_inputB;
                    // std::cout << " - OutA: " << RuntimeInfo.outputs.outputA;
                    // std::cout << " - OutB: " << RuntimeInfo.outputs.outputB << std::endl;
                    // std::cout << "----------------------------------" << std::endl;
                }
                else
                {
                    // usleep(10e6);
                    std::cout << "Runtime info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
                    exit(-1);
                }

                xControllerConfig.controller_mode_update = SJC_OPEN_LOOP_CONTROL;
                xControllerConfig.controller_ref = 0.0;

                if (SJC_RET_OK == Sjc.setControllerConfig(xControllerConfig))
                {
                }
                else
                {
                    std::cout << "Contoller configuration failed! \nPlease check the modbuss slave address..." << std::endl;
                    exit(-1);
                }
            }

            auto toc = std::chrono::high_resolution_clock::now();

            std::cout << "----------------------------------" << std::endl;
            std::cout << " * System Ts:  " << std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count()) << " ms." << std::endl;
            std::cout << " * Prev. Cycle duration: " << std::to_string(dur) << " ms." << std::endl;
            const auto end_cycle = std::chrono::high_resolution_clock::now();
            uint32_t dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_cycle - start).count();
            int TsTarget = 50;
            int delta = (TsTarget - dt) * 1e3;
            uint32_t wait = (delta < 0) ? TsTarget * 1e3 : delta;
            // uint32_t wait = (delta < 0) ? 0 : delta;
            std::cout << " * Wait time: " << std::to_string(wait / 1000) << " ms" << std::endl;
            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;
            usleep(wait);
        } while (1);
    }

    Sjc.~SjcInterface();

    return 0;
}