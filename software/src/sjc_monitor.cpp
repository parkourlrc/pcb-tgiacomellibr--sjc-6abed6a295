/**
 * @file sjc_monitor.cpp
 * @brief Multi-board monitoring tool with JSON configuration.
 *        Reads all sensor configurations, actuator configs, board info,
 *        and measurements from multiple SJC boards.
 *        Displays them in a tabulated format at 50 Hz.
 *
 * Usage:x
 *   ./sjc_monitor <config.json>
 */

#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include <nlohmann/json.hpp>
#include <sjc_realtime_interface.hpp>

// -----------------------------------------------------------------------------
// Signal handling for graceful shutdown
// -----------------------------------------------------------------------------
static volatile bool g_running = true;

void signal_handler(int signum)
{
    (void)signum;
    g_running = false;
}

// -----------------------------------------------------------------------------
// Board configuration structure
// -----------------------------------------------------------------------------
struct BoardConfig
{
    std::string name;
    std::string port;
    int address;
    int baudrate;
};

// -----------------------------------------------------------------------------
// Board runtime state
// -----------------------------------------------------------------------------
struct BoardState
{
    std::unique_ptr<SjcRealtimeInterface> interface;
    xSjcControlMeasInfo_t measurements{};
    xSjcSensorsConfig_t sensors{};
    xSjcBoardInfo_t board_info{};
    xSjcActuatorInfo_t actuator{};
    xSjcDigitalIOInfo_t digital_io{};
    bool connected{false};
    double read_time_ms{0.0};
    eSjcReturnType_t last_status{SJC_DISCONNECTED};
};

// -----------------------------------------------------------------------------
// Control mode to string
// -----------------------------------------------------------------------------
const char *control_mode_str(uint16_t mode)
{
    switch (mode)
    {
    case SJC_CONTROL_DISABLED:
        return "DISABLED";
    case SJC_OPEN_LOOP_CONTROL:
        return "OPEN_LOOP";
    case SJC_PID_SPEED_CONTROL:
        return "PID_SPEED";
    case SJC_PID_CURRENT_CONTROL:
        return "PID_CURR";
    case SJC_LQR_CURRENT_CONTROL:
        return "LQR_CURR";
    case SJC_DISCRETE_CURRENT_CONTROL:
        return "DISC_CURR";
    case SJC_SLOW_PRBS_EXCITATION:
        return "PRBS_SLOW";
    case SJC_MID_PRBS_EXCITATION:
        return "PRBS_MID";
    case SJC_FAST_PRBS_EXCITATION:
        return "PRBS_FAST";
    case SJC_SINE_PRBS_EXCITATION:
        return "SINE_PRBS";
    case SJC_PULSE_FAST_EXCITATION:
        return "PULSE_F";
    case SJC_PULSE_SLOW_EXCITATION:
        return "PULSE_S";
    case SJC_ANALOG_CALIBRATION:
        return "ANALOG_CAL";
    default:
        return "UNKNOWN";
    }
}

// -----------------------------------------------------------------------------
// Parse JSON configuration file
// -----------------------------------------------------------------------------
std::vector<BoardConfig> parse_config(const std::string &filename)
{
    std::vector<BoardConfig> configs;

    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open config file: " + filename);
    }

    nlohmann::json config;
    file >> config;

    if (!config.contains("boards"))
    {
        throw std::runtime_error("Config file must contain 'boards' array");
    }

    for (const auto &board : config["boards"])
    {
        BoardConfig cfg;
        cfg.name = board.value("name", "unnamed");
        cfg.port = board.at("port").get<std::string>();
        cfg.address = board.value("address", 1);
        cfg.baudrate = board.value("baudrate", 921600);
        configs.push_back(cfg);
    }

    return configs;
}

// -----------------------------------------------------------------------------
// Table formatting - dynamic column widths updated each cycle
// -----------------------------------------------------------------------------
struct TableFormat
{
    int col_label;
    int col_val;

    void update(const std::vector<BoardConfig> &configs, const std::vector<std::string> &all_values)
    {
        col_label = 16;
        col_val = 18; // minimum for "OPEN_LOOP (1000Hz)"

        // Check board names
        for (const auto &cfg : configs)
        {
            int len = static_cast<int>(cfg.name.length());
            if (len > col_val)
                col_val = len;
        }

        // Check all values to find the longest
        for (const auto &v : all_values)
        {
            int len = static_cast<int>(v.length());
            if (len > col_val)
                col_val = len;
        }

        col_val += 1; // padding
    }
};

static TableFormat g_fmt;

void print_separator(size_t num_boards)
{
    std::cout << std::string(g_fmt.col_label, '-');
    for (size_t i = 0; i < num_boards; ++i)
    {
        std::cout << "-+-" << std::string(g_fmt.col_val, '-');
    }
    std::cout << "\n";
}

void print_header_row(const std::vector<BoardConfig> &configs)
{
    std::cout << std::left << std::setw(g_fmt.col_label) << "Parameter";
    for (const auto &cfg : configs)
    {
        std::cout << " | " << std::setw(g_fmt.col_val) << cfg.name;
    }
    std::cout << "\n";
    print_separator(configs.size());
}

void print_section(const std::string &title)
{
    std::cout << "\n\033[1;36m" << title << "\033[0m\n";
}

template <typename T>
void print_row(const std::string &label, const std::vector<T> &values, const std::string &unit = "",
               int precision = 4)
{
    std::cout << std::left << std::setw(g_fmt.col_label) << label;
    for (const auto &val : values)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << val << unit;
        std::cout << " | " << std::right << std::setw(g_fmt.col_val) << oss.str();
    }
    std::cout << "\n";
}

void print_row_int(const std::string &label, const std::vector<int> &values, const std::string &unit = "")
{
    std::cout << std::left << std::setw(g_fmt.col_label) << label;
    for (const auto &val : values)
    {
        std::ostringstream oss;
        oss << val << unit;
        std::cout << " | " << std::right << std::setw(g_fmt.col_val) << oss.str();
    }
    std::cout << "\n";
}

void print_row_str(const std::string &label, const std::vector<std::string> &values)
{
    std::cout << std::left << std::setw(g_fmt.col_label) << label;
    for (const auto &val : values)
    {
        std::cout << " | " << std::right << std::setw(g_fmt.col_val) << val;
    }
    std::cout << "\n";
}

void print_row_bool(const std::string &label, const std::vector<bool> &values)
{
    std::cout << std::left << std::setw(g_fmt.col_label) << label;
    for (const auto &val : values)
    {
        std::cout << " | " << std::right << std::setw(g_fmt.col_val) << (val ? "ON" : "OFF");
    }
    std::cout << "\n";
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config.json>\n";
        return -1;
    }

    std::string config_file = argv[1];

    // Parse configuration
    std::vector<BoardConfig> configs;
    try
    {
        configs = parse_config(config_file);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing config: " << e.what() << "\n";
        return -1;
    }

    if (configs.empty())
    {
        std::cerr << "No boards configured in " << config_file << "\n";
        return -1;
    }

    std::cout << "Loaded " << configs.size() << " board(s) from " << config_file << "\n";

    // Initialize board interfaces
    std::vector<BoardState> boards(configs.size());

    for (size_t i = 0; i < configs.size(); ++i)
    {
        const auto &cfg = configs[i];
        std::cout << "Connecting to " << cfg.name << " at " << cfg.port << "...\n";

        boards[i].interface =
            std::make_unique<SjcRealtimeInterface>(cfg.port, cfg.address, cfg.baudrate);

        if (boards[i].interface->connect() == SJC_CONNECTED)
        {
            boards[i].connected = true;
            std::cout << "  [OK] Connected to " << cfg.name << "\n";

            // Read initial configs (these don't change frequently)
            boards[i].interface->getBoardInfo(&boards[i].board_info);
            boards[i].interface->getSensorsParameters(&boards[i].sensors);
            boards[i].interface->getActuatorParameters(&boards[i].actuator);
            boards[i].interface->getDigitalIOInfo(&boards[i].digital_io);

            // Start realtime thread
            boards[i].interface->start();
        }
        else
        {
            boards[i].connected = false;
            std::cerr << "  [FAIL] Could not connect to " << cfg.name << "\n";
        }
    }

    // Check if at least one board is connected
    bool any_connected = false;
    for (const auto &b : boards)
    {
        if (b.connected)
        {
            any_connected = true;
            break;
        }
    }

    if (!any_connected)
    {
        std::cerr << "No boards connected. Exiting.\n";
        return -1;
    }

    // Main monitoring loop at 50 Hz
    const int Ts_target_us = 20000; // 20 ms = 50 Hz
    int cycle = 0;
    int config_refresh_counter = 0;
    double last_cycle_ms = 0.0;

    while (g_running)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // Trigger read on all connected boards
        for (size_t i = 0; i < boards.size(); ++i)
        {
            if (boards[i].connected)
            {
                boards[i].interface->trigger_read();
            }
        }

        // Wait for all reads to complete
        for (size_t i = 0; i < boards.size(); ++i)
        {
            if (boards[i].connected)
            {
                boards[i].interface->wait_done();
                boards[i].measurements = boards[i].interface->get_runtime_info();
                boards[i].read_time_ms = boards[i].interface->get_last_read_ms();
                boards[i].last_status = boards[i].interface->get_last_read_status();
            }
        }

        // Refresh configs every 50 cycles (1 second)
        if (++config_refresh_counter >= 50)
        {
            config_refresh_counter = 0;
            for (size_t i = 0; i < boards.size(); ++i)
            {
                if (boards[i].connected)
                {
                    boards[i].interface->getBoardInfo(&boards[i].board_info);
                    boards[i].interface->getSensorsParameters(&boards[i].sensors);
                    boards[i].interface->getActuatorParameters(&boards[i].actuator);
                    boards[i].interface->getDigitalIOInfo(&boards[i].digital_io);
                }
            }
        }

        // Collect all string values for dynamic column width calculation
        std::vector<std::string> all_values;
        for (const auto &b : boards)
        {
            std::ostringstream oss;
            // Uptime
            oss.str("");
            oss << static_cast<int>(b.measurements.uptime / 1000) << "s (" << std::fixed << std::setprecision(1) << b.read_time_ms << "ms)";
            all_values.push_back(oss.str());
            // Ctrl Mode
            oss.str("");
            oss << control_mode_str(b.board_info.control_mode) << " (" << std::fixed << std::setprecision(0) << b.board_info.control_frequency << "Hz)";
            all_values.push_back(oss.str());
            // Positions
            oss.str("");
            oss << std::fixed << std::setprecision(4) << b.measurements.actuator_angular_position;
            all_values.push_back(oss.str());
        }
        g_fmt.update(configs, all_values);

        // Clear screen and print header with timing info
        std::cout << "\033[2J\033[H";
        std::cout << "\033[1;33m";
        std::cout << "SJC Monitor | Cycle: " << cycle++ << " | Last: " << std::fixed << std::setprecision(1)
                  << last_cycle_ms << "ms | Target: 20ms (50Hz) | Ctrl+C to exit\n";
        std::cout << "\033[0m";

        print_header_row(configs);

        // =================================================================
        // CONNECTION STATUS
        // =================================================================
        print_section("STATUS");
        {
            std::vector<std::string> status;
            for (const auto &b : boards)
            {
                if (!b.connected)
                    status.push_back("\033[31mDISCONN\033[0m");
                else if (b.last_status == SJC_RET_OK)
                    status.push_back("\033[32mOK\033[0m");
                else
                    status.push_back("\033[31mERROR\033[0m");
            }
            print_row_str("Status", status);
        }
        {
            // Uptime (Read Time)
            std::vector<std::string> vals;
            for (const auto &b : boards)
            {
                std::ostringstream oss;
                oss << static_cast<int>(b.measurements.uptime / 1000) << "s ("
                    << std::fixed << std::setprecision(1) << b.read_time_ms << "ms)";
                vals.push_back(oss.str());
            }
            print_row_str("Uptime (Rd)", vals);
        }

        // =================================================================
        // BOARD INFO
        // =================================================================
        print_section("BOARD INFO");
        {
            std::vector<std::string> vals;
            for (const auto &b : boards)
            {
                std::ostringstream oss;
                oss << (int)b.board_info.hardware_ver.major << "." << (int)b.board_info.hardware_ver.minor;
                vals.push_back(oss.str());
            }
            print_row_str("HW Version", vals);
        }
        {
            std::vector<std::string> vals;
            for (const auto &b : boards)
            {
                std::ostringstream oss;
                oss << (int)b.board_info.firmware_ver.major << "."
                    << (int)b.board_info.firmware_ver.minor << "."
                    << (int)b.board_info.firmware_ver.patch;
                vals.push_back(oss.str());
            }
            print_row_str("FW Version", vals);
        }
        {
            // Ctrl Mode (Frequency)
            std::vector<std::string> vals;
            for (const auto &b : boards)
            {
                std::ostringstream oss;
                oss << control_mode_str(b.board_info.control_mode) << " ("
                    << std::fixed << std::setprecision(0) << b.board_info.control_frequency << "Hz)";
                vals.push_back(oss.str());
            }
            print_row_str("Ctrl Mode", vals);
        }

        // =================================================================
        // ACTUATOR CONFIG
        // =================================================================
        print_section("ACTUATOR CONFIG");
        {
            std::vector<bool> vals;
            for (const auto &b : boards)
                vals.push_back(b.actuator.state);
            print_row_bool("Enabled", vals);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.actuator.system_voltage);
            print_row("Sys Voltage", vals, " V", 2);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.actuator.base_voltage);
            print_row("Base Voltage", vals, " V", 2);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.actuator.voltage_limit);
            print_row("Voltage Lim", vals, " V", 2);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.actuator.current_limit);
            print_row("Current Lim", vals, " A", 2);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.actuator.speed_limit);
            print_row("Speed Lim", vals, " rad/s", 2);
        }

        // =================================================================
        // SENSORS CONFIG (CPR = Counts Per Revolution, quadrature 4x)
        // =================================================================
        print_section("SENSORS CONFIG [CPR]");
        {
            std::vector<int> vals;
            for (const auto &b : boards)
                vals.push_back(b.sensors.actuator_encoder_resolution_cpr);
            print_row_int("Act Enc Res", vals, " cpr");
        }
        {
            std::vector<int> vals;
            for (const auto &b : boards)
                vals.push_back(b.sensors.swing_thx_encoder_resolution_cpr);
            print_row_int("THX Enc Res", vals, " cpr");
        }
        {
            std::vector<int> vals;
            for (const auto &b : boards)
                vals.push_back(b.sensors.swing_thy_encoder_resolution_cpr);
            print_row_int("THY Enc Res", vals, " cpr");
        }

        // =================================================================
        // DIGITAL I/O
        // =================================================================
        print_section("DIGITAL I/O");
        {
            std::vector<bool> vals;
            for (const auto &b : boards)
                vals.push_back(b.digital_io.inputs.switch_a);
            print_row_bool("Switch A", vals);
        }
        {
            std::vector<bool> vals;
            for (const auto &b : boards)
                vals.push_back(b.digital_io.inputs.switch_b);
            print_row_bool("Switch B", vals);
        }
        {
            std::vector<bool> vals;
            for (const auto &b : boards)
                vals.push_back(b.digital_io.inputs.iso_input_a);
            print_row_bool("Iso Input A", vals);
        }
        {
            std::vector<bool> vals;
            for (const auto &b : boards)
                vals.push_back(b.digital_io.inputs.iso_input_b);
            print_row_bool("Iso Input B", vals);
        }
        {
            std::vector<bool> vals;
            for (const auto &b : boards)
                vals.push_back(b.digital_io.outputs.output_a);
            print_row_bool("Output A", vals);
        }
        {
            std::vector<bool> vals;
            for (const auto &b : boards)
                vals.push_back(b.digital_io.outputs.output_b);
            print_row_bool("Output B", vals);
        }

        // =================================================================
        // POSITION MEASUREMENTS
        // =================================================================
        print_section("POSITIONS [rad]");
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.actuator_angular_position);
            print_row("Actuator", vals, "", 4);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.swing_angular_position_thx);
            print_row("Swing THX", vals, "", 4);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.swing_angular_position_thy);
            print_row("Swing THY", vals, "", 4);
        }

        // =================================================================
        // VELOCITY MEASUREMENTS
        // =================================================================
        print_section("VELOCITIES [rad/s]");
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.actuator_angular_velocity);
            print_row("Actuator", vals, "", 3);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.swing_angular_velocity_thx);
            print_row("Swing THX", vals, "", 3);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.swing_angular_velocity_thy);
            print_row("Swing THY", vals, "", 3);
        }

        // =================================================================
        // ELECTRICAL MEASUREMENTS
        // =================================================================
        print_section("ELECTRICAL");
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.actuator_voltage);
            print_row("Voltage", vals, " V", 3);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.actuator_current);
            print_row("Current", vals, " A", 4);
        }

        // =================================================================
        // CONTROLLER STATE
        // =================================================================
        print_section("CONTROLLER");
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.controller_ref);
            print_row("Reference", vals, "", 4);
        }
        {
            std::vector<double> vals;
            for (const auto &b : boards)
                vals.push_back(b.measurements.controller_error);
            print_row("Error", vals, "", 4);
        }

        // Timing control
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        int delay_us = std::max(0L, Ts_target_us - elapsed_us);
        last_cycle_ms = elapsed_us / 1000.0;

        usleep(delay_us);
    }

    // Cleanup
    std::cout << "\nShutting down...\n";

    for (size_t i = 0; i < boards.size(); ++i)
    {
        if (boards[i].connected)
        {
            boards[i].interface->stop();
            boards[i].interface->disconnect();
            std::cout << "Disconnected from " << configs[i].name << "\n";
        }
    }

    std::cout << "Done.\n";
    return 0;
}
