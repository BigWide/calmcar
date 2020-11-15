#pragma once
#ifdef __linux__
#include <arpa/inet.h>
#include <unistd.h>
#endif
#ifdef WIN32
#include <WinSock2.h>
#endif
#include <string>

namespace calmcar_perception
{
class TcpConnection
{
  public:
    TcpConnection();
    ~TcpConnection();
    TcpConnection* Accept();
    int ConnectTo(const char* ip, short port);
    int Write(void* buff, size_t len);
    int Read(void* buff, size_t len);
    void Close();
    bool Bind(char* ip, short port);

  private:
    TcpConnection(int socket, sockaddr_in& addr);
    TcpConnection(const TcpConnection&);
    TcpConnection& operator=(const TcpConnection&);

    int socket_;
    std::string ip_;
    short port_;
};
}  // namespace calmcar_perception
