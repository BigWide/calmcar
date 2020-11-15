/*
 * tcp_client.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: chengsq
 */
#include "tcp_client.h"
#include <string.h>
#include "tcp_connection.h"

namespace calmcar_perception
{

char* command_jpeg = (char*)"JPEG FORMAT QUALITY 75\r\n";
char* command_get_info = (char*)"GET INFO\r\n";
char* command_start = (char*)"START\r\n";
static const int FRAME_LENGHT = 4096000;

TcpClient::TcpClient()
{
    tcp_ = NULL;
    height_ = 0;
    width_ = 0;
    frame_length_ = 0;
    pixel_format_ = -1;
    read_buffer_ = NULL;
}

TcpClient::~TcpClient()
{
    // TODO Auto-generated destructor stub
}

/*获取图像宽度 */
int TcpClient::GetWidth() const
{
    return width_;
}

/*获取图像高度  */
int TcpClient::GetHeight() const
{
    return height_;
}

/*获取图像格式  */
int TcpClient::GetPixelFormat() const
{
    return pixel_format_;
}

/*获取车辆类型，暂时无用  */
int TcpClient::GetCarType() const
{
    return car_type_;
}

/*获取can版本信息，暂时无用  */
int TcpClient::GetCanVersion() const
{
    return can_version_;
}

/* 设置使用功能，目前只有两个功能
 * 1、获取原始图像
 * 2、获取JPEG压缩后的图像数据
 * */
void TcpClient::SetFunction(int f)
{
    func_ = f;
}

/* 建立网络连接
 * ip:为设备的网络地址IPv4
 * port:为设备的端口*/
int TcpClient::ConnectTo(const char* ip, int port)
{
    tcp_ = new TcpConnection();
    int ret = tcp_->ConnectTo(ip, port);
    if (ret == -1)
        return -1;
    char msg[100];
    int len = tcp_->Read(msg, sizeof(msg));
    if (func_ == BSD)
    {
        tcp_->Write(command_jpeg, strlen(command_jpeg));
    }
    else
    {
        tcp_->Write(command_get_info, strlen(command_get_info));
    }

    len = tcp_->Read(msg, sizeof(msg));
    if (len <= 0)
        return -1;
    msg[len] = 0;
    char pixel_format_s[10];
    sscanf(msg,
           "WIDTH %d HEIGHT %d FORMAT %s CAN VERSION %d CAR_TYPE %d",
           &width_,
           &height_,
           pixel_format_s,
           &can_version_,
           &car_type_);
    printf("%s \n", msg);
    tcp_->Write(command_start, strlen(command_start));
    std::string pixel_format_cstring = pixel_format_s;
    if (pixel_format_cstring == "YUV420")
    {
        pixel_format_ = PIXEL_FORMAT_YUV420;
        frame_length_ = width_ * height_ * 1.5;
    }
    else if (pixel_format_cstring == "JPEG")
    {
        pixel_format_ = PIXEL_FORMAT_JPEG;
    }
    read_buffer_ = new char[FRAME_LENGHT * 2];
    frame_length_ = FRAME_LENGHT * 2;
}

/*更新数据 */
int TcpClient::Update()
{
    char package_header[9] = {'\0'};
    int curent_frame_length = 0;
    int head_len = 12;
    read_length_ = 0;
    while (read_length_ < head_len)
    {
        int size = tcp_->Read(read_buffer_ + read_length_, head_len - read_length_);
        if (size <= 0)
            return -1;
        read_length_ += size;
    }
    memcpy(package_header, read_buffer_, 8);
    memcpy(&curent_frame_length, read_buffer_ + 8, 4);
    if (strncmp(package_header, "calmcar:", 8) != 0)
    {
        printf("head:%s %d \n", package_header, curent_frame_length);
        return -1;
    }
    if (frame_length_ < curent_frame_length)
    {
        delete read_buffer_;
        read_buffer_ = new char[curent_frame_length];
        frame_length_ = curent_frame_length;
    }
    read_length_ = 0;
    while (read_length_ < curent_frame_length)
    {
        int size = tcp_->Read(read_buffer_ + read_length_, curent_frame_length - read_length_);
        read_length_ += size;
        if (size < 0)
            return -1;
    }
    return read_length_;
}

/*获取更新后的数据  */
char* TcpClient::GetData() const
{
    return read_buffer_;
}

/*关闭网络连接  */
void TcpClient::Close()
{
    tcp_->Close();
    delete tcp_;
    tcp_ = NULL;
}
}  // namespace calmcar_perception
