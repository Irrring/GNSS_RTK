// RTKVisualServer.h
#ifndef RTK_VISUAL_SERVER_H
#define RTK_VISUAL_SERVER_H

#include "my_rtk.hpp"
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <ws2tcpip.h> // 添加该头文件以支持 inet_pton

class RTKVisualServer {
private:
    SOCKET serverSocket = INVALID_SOCKET;
    SOCKET clientSocket = INVALID_SOCKET;
    std::string ip;
    int port;
    std::thread serverThread;
    std::atomic<bool> running{ false };
    std::mutex dataMutex;
    std::queue<std::string> dataQueue;

public:
    RTKVisualServer(const std::string& ip, int port) : ip(ip), port(port) {}

    ~RTKVisualServer() {
        stop();
    }

    bool start() {
        // 初始化 Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "Failed to initialize Winsock" << std::endl;
            return false;
        }

        // 创建服务器套接字
        serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (serverSocket == INVALID_SOCKET) {
            std::cerr << "Failed to create server socket" << std::endl;
            WSACleanup();
            return false;
        }

        // 设置服务器地址
        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(port);
        // 使用 inet_pton 代替 inet_addr
        if (inet_pton(AF_INET, ip.c_str(), &(serverAddr.sin_addr)) != 1) {
            std::cerr << "Invalid address or address not supported" << std::endl;
            closesocket(serverSocket);
            WSACleanup();
            return false;
        }
        memset(serverAddr.sin_zero, 0, sizeof(serverAddr.sin_zero));

        // 绑定套接字 - 修复 bind 函数调用
        if (::bind(serverSocket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR) {
            std::cerr << "Bind failed with error: " << WSAGetLastError() << std::endl;
            closesocket(serverSocket);
            WSACleanup();
            return false;
        }

        // 监听连接
        if (listen(serverSocket, 1) == SOCKET_ERROR) {
            std::cerr << "Listen failed with error: " << WSAGetLastError() << std::endl;
            closesocket(serverSocket);
            WSACleanup();
            return false;
        }

        running = true;
        std::cout << "RTK Visual TCP Server started on " << ip << ":" << port << std::endl;

        // 启动服务器线程
        serverThread = std::thread(&RTKVisualServer::serverLoop, this);

        return true;
    }

    void stop() {
        running = false;

        if (clientSocket != INVALID_SOCKET) {
            closesocket(clientSocket);
            clientSocket = INVALID_SOCKET;
        }

        if (serverSocket != INVALID_SOCKET) {
            closesocket(serverSocket);
            serverSocket = INVALID_SOCKET;
        }

        WSACleanup();

        if (serverThread.joinable()) {
            serverThread.join();
        }

        std::cout << "RTK Visual TCP Server stopped" << std::endl;
    }

    // 添加数据到发送队列
    void addData(const std::string& data) {
        std::lock_guard<std::mutex> lock(dataMutex);
        dataQueue.push(data);
    }

private:
    void serverLoop() {
        while (running) {
            // 接受客户端连接
            if (clientSocket == INVALID_SOCKET) {
                std::cout << "Waiting for RTKPlot to connect..." << std::endl;
                clientSocket = accept(serverSocket, nullptr, nullptr);
                if (clientSocket == INVALID_SOCKET) {
                    if (running) {
                        std::cerr << "Accept failed with error: " << WSAGetLastError() << std::endl;
                        Sleep(1000);  // 避免快速循环消耗CPU
                    }
                    continue;
                }
                std::cout << "RTKPlot client connected!" << std::endl;
            }

            // 检查队列中是否有数据需要发送
            std::string dataToSend;
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                if (!dataQueue.empty()) {
                    dataToSend = dataQueue.front();
                    dataQueue.pop();
                }
            }

            // 发送数据
            if (!dataToSend.empty()) {
                int result = send(clientSocket, dataToSend.c_str(), dataToSend.length(), 0);
                if (result == SOCKET_ERROR) {
                    std::cerr << "Send failed with error: " << WSAGetLastError() << std::endl;
                    closesocket(clientSocket);
                    clientSocket = INVALID_SOCKET;
                }
            }
            else {
                // 没有数据，稍微休息一下
                Sleep(10);
            }
        }
    }
};

#endif // RTK_VISUAL_SERVER_H
