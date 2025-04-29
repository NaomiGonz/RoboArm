#include <iostream>
#include <string>
#include <thread>         
#include <chrono>         
#include <atomic>
#include <csignal>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>

// Aliases
namespace asio = boost::asio;
namespace po = boost::program_options;

// --- Global Variables ---
asio::io_context io_ctx; 
asio::serial_port serial_port(io_ctx);
std::atomic<bool> running(true); 

// --- Signal Handler for Ctrl+C ---
void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cerr << "\nCtrl+C detected. Shutting down..." << std::endl;
        running = false;
        boost::system::error_code ignored_ec;
        if (serial_port.is_open()) {
            serial_port.cancel(ignored_ec);
        }
    }
}

// --- Main Function ---
int main(int argc, char* argv[]) {
    std::string port_name;
    const int read_delay_seconds = 15; // Delay of read after write in secs

    // --- Argument Parsing ---
    try {
        po::options_description desc("Serial Communication with Delayed Read Options");
        desc.add_options()
            ("help,h", "Produce help message")
            ("port,p", po::value<std::string>(&port_name)->required(), "Serial port name (e.g., COM1 or /dev/ttyUSB0)");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 1;
        }

        po::notify(vm); // Check for required options

    } catch (const po::error& e) {
        std::cerr << "Error parsing command line: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // --- Register Signal Handler ---
    signal(SIGINT, signal_handler);

    // --- Serial Port Setup ---
    boost::system::error_code ec; 

    try {
        // Open the serial port
        serial_port.open(port_name, ec);
        if (ec) {
            std::cerr << "Error opening serial port " << port_name << ": " << ec.message() << std::endl;
            return 1;
        }

        // Set serial port configurations 
        serial_port.set_option(asio::serial_port_base::baud_rate(115200), ec);
        if (ec) { std::cerr << "Error setting baud rate: " << ec.message() << std::endl; serial_port.close(); return 1; }
        serial_port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none), ec);
        if (ec) { std::cerr << "Error setting flow control: " << ec.message() << std::endl; serial_port.close(); return 1; }
        serial_port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none), ec);
        if (ec) { std::cerr << "Error setting parity: " << ec.message() << std::endl; serial_port.close(); return 1; }
        serial_port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one), ec);
        if (ec) { std::cerr << "Error setting stop bits: " << ec.message() << std::endl; serial_port.close(); return 1; }
        serial_port.set_option(asio::serial_port_base::character_size(8), ec);
        if (ec) { std::cerr << "Error setting character size: " << ec.message() << std::endl; serial_port.close(); return 1; }

        std::cout << "Serial port " << port_name << " opened successfully." << std::endl;
        std::cout << "Enter commands to send. Read attempt will follow "
                  << read_delay_seconds << "s after each send." << std::endl;
        std::cout << "(Ctrl+C to exit)" << std::endl;

        // --- Main Loop for Writing and Delayed Reading ---
        std::string command;
        asio::streambuf read_buf; 

        while (running) {
            std::cout << "> "; 

            // Read command from user input
            if (!std::getline(std::cin, command)) {
                 if (std::cin.eof()) { 
                    std::cout << "ERROR: Input EOF detected. Shutting down." << std::endl;
                 } else if (std::cin.bad()) {
                    std::cerr << "ERROR: Input stream error. Shutting down." << std::endl;
                 } else {
                    std::cerr << "ERROR: Input error. Shutting down." << std::endl;
                 }
                 running = false; // Signal shutdown
                 break; 
            }

            if (!running) break; 

            // --- Write Command ---
            command += '\n'; // Append newline
            asio::write(serial_port, asio::buffer(command), ec);

            // Error check
            if (ec) {
                std::cerr << "Error writing to serial port: " << ec.message() << std::endl;
                running = false; // Signal shutdown 
                break; 
            }

            std::cout << "Write successful. Waiting " << read_delay_seconds << " seconds before reading..." << std::endl;

            // Wait 15 secs, check the running flag periodically during sleep
            for (int i = 0; i < read_delay_seconds && running; ++i) {
                 std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (!running) break; 

            // --- Attempt to Read ---
            std::cout << "Attempting to read..." << std::endl;
            read_buf.consume(read_buf.size()); // Clear buffer before reading
            ec.clear(); // Clear previous error codes before read attempt

            // Blocking read until newline
            size_t bytes_read = asio::read_until(serial_port, read_buf, '\n', ec);

            if (!running) break; 

            if (ec) {
                // Handle read errors
                if (ec == asio::error::operation_aborted) {
                     std::cerr << "[Read] Operation cancelled (likely shutdown)." << std::endl;
                } else if (ec == asio::error::eof) {
                     std::cerr << "[Read] EOF received." << std::endl;
                } else {
                    std::cerr << "[Read] Error: " << ec.message() << std::endl;
                    running = false;
                    break;
                }
            } else if (bytes_read > 0) {

                // Successfully read a line
                std::istream is(&read_buf);
                std::string line;
                std::getline(is, line); 

                // Process or print the received line
                std::cout << "[Received] " << line << std::endl;
            } else {
                 // read_until returned 0 bytes without error (unlikely but possible)
                 std::cout << "[Read] No data received before delimiter." << std::endl;
            }
            
        }

    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception in main: " << e.what() << std::endl;
        // Ensure port is closed if an exception occurred after opening
        boost::system::error_code ignored_ec;
        if (serial_port.is_open()) {
            serial_port.close(ignored_ec);
        }
        return 1;
    }

    // --- Cleanup ---
    std::cout << "Exiting program..." << std::endl;
    boost::system::error_code ignored_ec;
    if (serial_port.is_open()) {
        serial_port.close(ignored_ec); // Ignore errors on close during shutdown
        std::cout << "Serial port closed." << std::endl;
    }

    return 0;
}
