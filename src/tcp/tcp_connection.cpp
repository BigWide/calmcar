#include "tcp_connection.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef __linux__
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#else
#include <WinSock2.h>
#include <ws2tcpip.h>
#endif

namespace calmcar_perception
{

TcpConnection::TcpConnection()
{
    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int on = 1;
#ifdef __linux__
    setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE, (void*)&on, sizeof(on));
    setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, (void*)&on, sizeof(on));
#else
    setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE, (const char*)&on, sizeof(on));
    setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, (const char*)&on, sizeof(on));
#endif
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
#ifdef __linux__
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
#else
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) == -1)
#endif
    {
        printf("set socket SO_RCVTIMEO error\n");
    }
    port_ = 0;
}

TcpConnection::TcpConnection(int socket, sockaddr_in& addr)
{
    socket_ = socket;
    ip_ = inet_ntoa(addr.sin_addr);
    port_ = ntohl(addr.sin_port);
}

TcpConnection::~TcpConnection()
{
    Close();
}
bool TcpConnection::Bind(char* ip, short port)
{
    sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port = htons(port);
    if (!ip)
        sin.sin_addr.s_addr = htonl(INADDR_ANY);
    else if (strcmp(ip, "127.0.0.1") == 0)
        sin.sin_addr.s_addr = htonl(INADDR_ANY);
    else
        sin.sin_addr.s_addr = inet_addr(ip);
    if (bind(socket_, (sockaddr*)&sin, sizeof(sin)))
    {
        return false;
    }
    if (listen(socket_, 5))
    {
        return false;
    }
    return true;
}

TcpConnection* TcpConnection::Accept()
{
    int sClient;
    sockaddr_in remote_addr;
    socklen_t addr_len = sizeof(remote_addr);
    sClient = accept(socket_, (sockaddr*)&remote_addr, &addr_len);
    if (sClient < 0)
    {
        return NULL;
    }
    else
    {
        TcpConnection* new_socket = new TcpConnection(sClient, remote_addr);
        return new_socket;
    }
}

/* 建立网络连接函数
 * ip:目标设备的ip地址
 * Port:目标射比的端口号*/
int TcpConnection::ConnectTo(const char* ip, short Port)
{
    sockaddr_in serAddr;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(Port);
    serAddr.sin_addr.s_addr = inet_addr(ip);
    socklen_t len = sizeof(serAddr);
    if (connect(socket_, (sockaddr*)&serAddr, len) < 0)
    {
        return -1;
    }

    return 0;
}

/* 写网络数据函数
 * buff:要写的数据地址
 * len:要写的数据长度*/
int TcpConnection::Write(void* buff, size_t len)
{
#ifdef __linux__
    int err = ::send(socket_, (char*)buff, len, MSG_NOSIGNAL);
#else
    int err = ::send(socket_, (char*)buff, len, 0);
#endif
    return err < 0 ? -1 : err;
}

/* 读网络数据函数
 * buff:保存读取数据的内存地址
 * len:要读取的数据的长度*/
int TcpConnection::Read(void* buff, size_t len)
{
#ifdef __linux__
    int err = ::recv(socket_, (char*)buff, len, MSG_NOSIGNAL);
#else
    int err = ::recv(socket_, (char*)buff, len, 0);
#endif
    int enable = 1;
#ifdef __linux__
    setsockopt(socket_, IPPROTO_TCP, TCP_QUICKACK, (void*)&enable, sizeof(enable));
#else
    setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, (const char*)&enable, sizeof(int));
#endif
    return err < 0 ? -1 : err;
}

/* 关闭网络连接 */
void TcpConnection::Close()
{
    if (socket_ >= 0)
#ifdef __linux__
        close(socket_);
#else
        closesocket(socket_);
#endif
    socket_ = -1;
}
}  // namespace calmcar_perception
