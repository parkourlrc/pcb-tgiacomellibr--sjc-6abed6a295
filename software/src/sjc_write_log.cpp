/**
 * @file sjc_write_log.cpp
 * @brief Standalone serial logger: reads JSON lines from serial and saves to CSV.
 *
 * Flow:
 *   - Opens the configured serial port.
 *   - Reads JSON lines.
 *   - Converts to CSV.
 *   - Creates output directories automatically if they don't exist.
 *
 * Usage:
 *   ./sjc_write_log <serial_port> <baud_rate> <log_file>
 *
 * Example:
 *   ./sjc_write_log /dev/ttyACM1 4000000 /home/crane/Logs/joints/Bridge-ol_5V_50Hz_2025-09-13_Take-1.csv
 */

#include <nlohmann/json.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

class SJCLogger {
public:
    SJCLogger(const std::string &port, int baud, const std::string &file)
        : serial_port_(port), baud_rate_(baud), log_file_(file) {}

    bool run() {
        if (!open_serial())
            return false;

        if (!setup_logging())
            return false;

        std::string buffer;
        std::cout << "[INFO] Waiting for JSON lines on " << serial_port_ << " ..." << std::endl;

        while (true) {
            if (read_one_char(buffer)) {
                parse_and_log(buffer);
                buffer.clear();
            }
        }

        return true;
    }

private:
    std::string serial_port_;
    int baud_rate_;
    std::string log_file_;
    std::ofstream log_stream_;
    int fd_ = -1;

    // ===============================
    // Serial setup
    // ===============================
    bool open_serial() {
        fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "[ERROR] Failed to open serial port: " << serial_port_ << std::endl;
            return false;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "[ERROR] Failed to get termios attributes" << std::endl;
            return false;
        }

        cfsetospeed(&tty, baud_rate_);
        cfsetispeed(&tty, baud_rate_);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "[ERROR] Failed to set termios attributes" << std::endl;
            return false;
        }

        tcflush(fd_, TCIFLUSH);

        std::cout << "[INFO] Serial port opened: " << serial_port_
                  << " @ " << baud_rate_ << " baud" << std::endl;
        return true;
    }

    // ===============================
    // Logging setup
    // ===============================
    bool setup_logging() {
        try {
            fs::path full_path = log_file_;
            fs::create_directories(full_path.parent_path());

            log_stream_.open(full_path, std::ios::out | std::ios::trunc);
            if (!log_stream_)
                throw std::runtime_error("Failed to open file: " + full_path.string());

            log_stream_ << "t,i,v,vs,pos,vel,rpm,r,e,aux01,aux02\n";
            std::cout << "[INFO] CSV file created: " << full_path << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[ERROR] Log file creation failed: " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    // ===============================
    // Serial read helpers
    // ===============================
    bool read_one_char(std::string &buffer) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);
        timeval timeout{0, 10000}; // 10 ms

        if (select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout) > 0) {
            char c;
            if (read(fd_, &c, 1) > 0) {
                if (c == '\r' || c == '\n') {
                    if (!buffer.empty())
                        return true;
                } else if (c >= 32 && c <= 126) {
                    buffer += c;
                }
            }
        }
        return false;
    }

    // ===============================
    // JSON parsing + CSV logging
    // ===============================
    void parse_and_log(const std::string &line) {
        try {
            auto data = nlohmann::json::parse(line);

            if (!(data.contains("t") && data.contains("i") && data.contains("v") &&
                  data.contains("vs") && data.contains("pos") && data.contains("vel") &&
                  data.contains("rpm") && data.contains("r") && data.contains("e") &&
                  data.contains("aux01") && data.contains("aux02"))) {
                std::cerr << "[WARN] Missing fields in JSON: " << line << std::endl;
                return;
            }

            log_stream_ << data["t"] << "," << data["i"] << "," << data["v"] << ","
                        << data["vs"] << "," << data["pos"] << "," << data["vel"] << ","
                        << data["rpm"] << "," << data["r"] << "," << data["e"] << ","
                        << data["aux01"] << "," << data["aux02"] << "\n";
        } catch (const std::exception &e) {
            std::cerr << "[WARN] JSON parse failed: " << e.what()
                      << " | line=" << line << std::endl;
        }
    }
};

// ===============================
// Main
// ===============================
int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <baud_rate> <log_file>\n";
        return 1;
    }

    std::string serial_port = argv[1];
    int baud_rate = std::stoi(argv[2]);
    std::string log_file = argv[3];

    SJCLogger logger(serial_port, baud_rate, log_file);
    return logger.run() ? 0 : 1;
}
