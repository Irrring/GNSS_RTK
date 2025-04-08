#include "my_rtk.hpp"
#include <iostream>
#include <stdexcept>
#include <cstdio>
#include <winsock2.h>  // 用于 SOCKET 类型的支持

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)


// 构造函数实现
InputHandle::InputHandle(const char* filepath) : type_(InputType::FILE_INPUT) {
    fp_ = fopen(filepath, "rb");
    if (!fp_) throw std::runtime_error("File open failed");
}

InputHandle::InputHandle(const char IP[], const unsigned short Port) : type_(InputType::SOCKET_INPUT) {
    if (!OpenSocket(sock_, IP, Port))
        throw std::runtime_error("Socket open failed");
}

// 析构函数实现
InputHandle::~InputHandle() {
    close_resource();
}

// 读取接口实现
int InputHandle::read() {
    if (!msg_buff) return -1;

    int bytesRead = 0;

    switch (type_) {
    case InputType::FILE_INPUT:
        if (!fp_) return -1;
        bytesRead = fread(reinterpret_cast<char*>(buff_ptr + buff_len), 1, MSG_BUFF_SIZE - buff_len, fp_);
        buff_len += bytesRead;
        valid_size = buff_len;
        return bytesRead;

    case InputType::SOCKET_INPUT:
        bytesRead = recv(sock_, reinterpret_cast<char*>(buff_ptr + buff_len), MSG_BUFF_SIZE - buff_len, 0);
        if (bytesRead == SOCKET_ERROR || bytesRead == 0) {
            std::cerr << "Error in receiving GNSS data or connection closed." << std::endl;
            return -1;
        }
        buff_len += bytesRead;
        valid_size = buff_len;
        return bytesRead;

    default:
        return -1;
    }
}

// 资源关闭方法实现
void InputHandle::close_resource() {
    switch (type_) {
    case InputType::FILE_INPUT:
        if (fp_)
            fclose(fp_);
        fp_ = nullptr;
        break;

    case InputType::SOCKET_INPUT:
        if (sock_ != INVALID_SOCKET)
            CloseSocket(sock_);
        sock_ = INVALID_SOCKET;
        break;

    default: break;
    }
    type_ = InputType::NONE;
}

// 打开套接字实现
bool InputHandle::OpenSocket(SOCKET& sock, const char* IP, unsigned short port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed" << std::endl;
        return false;
    }

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed" << std::endl;
        return false;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(IP);

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "Connection failed" << std::endl;
        return false;
    }

    return true;
}

// 关闭套接字实现
void InputHandle::CloseSocket(SOCKET sock) {
    closesocket(sock);
    WSACleanup();
}
