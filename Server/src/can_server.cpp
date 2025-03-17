#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <vector>
#include <sstream>
#include <memory>
#include <array>

class CANServer {
private:
    int server_fd;
    int port;
    std::vector<std::thread> client_threads;

    std::string executeCommand(const std::string& cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
        
        if (!pipe) {
            return "Error executing command";
        }
        
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        
        return result;
    }

    void handleClient(int client_socket) {
        char buffer[1024] = {0};
        while (true) {
            memset(buffer, 0, sizeof(buffer));
            int valread = read(client_socket, buffer, 1024);
            if (valread <= 0) break;

            std::string command(buffer);
            std::string response;

            if (command.find("SEND:") == 0) {
                // Format: "SEND:01#F6010ADC0A006B"
                std::string can_msg = command.substr(5);
                std::string cmd = "cansend slcan0 " + can_msg;
                response = executeCommand(cmd);
            }
            else if (command == "MONITOR") {
                std::string cmd = "candump slcan0 -T 1000 -n 1";
                response = executeCommand(cmd);
            }

            send(client_socket, response.c_str(), response.length(), 0);
        }
        close(client_socket);
    }

public:
    CANServer(int port = 5000) : port(port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            throw std::runtime_error("Socket creation failed");
        }

        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            throw std::runtime_error("Bind failed");
        }
    }

    void start() {
        if (listen(server_fd, 3) < 0) {
            throw std::runtime_error("Listen failed");
        }

        std::cout << "Server listening on port " << port << std::endl;

        while (true) {
            sockaddr_in client_addr;
            socklen_t addrlen = sizeof(client_addr);
            int client_socket = accept(server_fd, (struct sockaddr *)&client_addr, &addrlen);
            
            if (client_socket < 0) {
                std::cerr << "Accept failed" << std::endl;
                continue;
            }

            client_threads.emplace_back(&CANServer::handleClient, this, client_socket);
        }
    }

    ~CANServer() {
        close(server_fd);
        for (auto& thread : client_threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }
};

int main() {
    try {
        CANServer server;
        server.start();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
} 