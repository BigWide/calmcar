/*
 * tcp_client.h
 *
 *  Created on: Jun 3, 2017
 *      Author: chengsq
 */

#ifndef SRC_TCP_TCP_CLIENT_H_
#define SRC_TCP_TCP_CLIENT_H_
#include <vector>
//#define PIXEL_FORMAT_YUV422 (1)
//#define PIXEL_FORMAT_RGB888 (2)
//#define PIXEL_FORMAT_YUV420 (3)
//#define PIXEL_FORMAT_JPEG (4)

namespace calmcar_perception
{

enum
{
    FCM = 0,
    BSD = 1,
    VBOX = 2
};
enum
{
    PIXEL_FORMAT_JPEG = 4,
    PIXEL_FORMAT_YUV420 = 3
};

class TcpConnection;
class TcpClient
{
  public:
    TcpClient();
    int ConnectTo(const char* ip, int port);
    void SetFunction(int f);
    int Update();
    int GetWidth() const;
    int GetHeight() const;
    int GetPixelFormat() const;
    int GetCanVersion() const;
    int GetCarType() const;
    char* GetData() const;
    void Close();
    virtual ~TcpClient();

  private:
    TcpConnection* tcp_;
    char* read_buffer_;
    int frame_length_;
    int pixel_format_;
    int width_;
    int height_;
    int read_length_;
    int can_version_;
    int car_type_;
    int func_;
};
}  // namespace calmcar_perception
#endif /* SRC_TCP_TCP_CLIENT_H_ */
