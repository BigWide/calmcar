#include "calmcar_sdk.h"
#include <string.h>
#include <iostream>
#include <error.h>
#include <unistd.h>
#ifdef __linux__
#include <unistd.h>
#else
#include <windows.h>
#endif

namespace calmcar_perception
{

CalmcarSdk::CalmcarSdk()
{
    stop = false;
    format_ = 0;
}

CalmcarSdk::~CalmcarSdk()
{
    stop = true;
    if (mode == ONLINE)
    {
        client_->Close();
        delete client_;
    }
    while (buf.size() > 0)
    {
        buf.pop_front();
    }
    if (proto_buffer)
        delete proto_buffer;
    if (read_buf)
        delete read_buf;
}

/*获取一帧protobuf数据  */
ProtobufFrame CalmcarSdk::GetData()
{
    std::vector< char > fPair;
    ProtobufFrame frame;
    fPair = buf[0];
    frame.ParseFromArray(&fPair[0], fPair.size());
    buf.pop_front();
    return frame /* something */;
}

/* 初始化函数
 * src:可以为目标设备的IP地址，也可以是PDAQ录制好的视频数据
 * port:目标设备的端口
 * func: 指定使用的功能*/
bool CalmcarSdk::Init(std::string src, int port, int func)
{
    bool ret = false;
    if (src.find("mp4") != std::string::npos)
    {
        ret = InitOffline(src);
        mode = OFFLINE;
    }
    else
    {
        func_ = func;
        ret = InitOnline(src, port);
        mode = ONLINE;
    }
    return ret;
}

/* 离线仿真模式初始化函数
 * file_name:PDQA录制的视频文件的绝对路径*/
bool CalmcarSdk::InitOffline(std::string file_name)
{
    video_file_name_ = file_name;
    csd_file_name_ = file_name;
    if (::access(video_file_name_.c_str(), F_OK) < 0)
    {
        fprintf(stderr, "%s\n", strerror(errno));
        return false;
    }

    csd_file_name_.replace(csd_file_name_.find_last_of('m'), 3, "csd");
    if (::access(csd_file_name_.c_str(), F_OK) < 0)
    {
        fprintf(stderr, "%s\n", strerror(errno));
        return false;
    }
    capture = new cv::VideoCapture();
    capture->open(video_file_name_);
    if (!capture->isOpened())
    {
        fprintf(stderr, "open video file error\n");
        return false;
    }
    in_stream.open(csd_file_name_, std::ios::binary | std::ios::in);
    if (!in_stream.is_open())
    {
        fprintf(stderr, "open csd file error\n");
        return false;
    }
    format_ = 0;
}

/* 在线模式初始化函数
 * dst:目标设备的ip地址
 * p:目标设备的端口号*/
bool CalmcarSdk::InitOnline(std::string dst, int p)
{
    int ret = 0;
    ip = dst;
    port = p;
    client_ = new TcpClient();
    client_->SetFunction(func_);
    ret = client_->ConnectTo(dst.c_str(), port);
    if (ret == -1)
        return false;
    width_ = client_->GetWidth();
    height_ = client_->GetHeight();
    format_ = client_->GetPixelFormat();
    return true;
}

/* 更新数据 */
bool CalmcarSdk::Update()
{
    if (mode == ONLINE)
    {
        return OnlineUpdate();
    }
    else
    {
        return OfflineUpdate();
    }
}

/* 在线模式更新数据 */
bool CalmcarSdk::OnlineUpdate()
{
    int res = -1;
    bool ret = false;

    for (int try_count = 0; try_count < 5; ++try_count)
    {
        if (stop)
            break;
        if (try_count > 5)
        {
            ret = false;
        }
        res = client_->Update();
        if (res == 0)
        {
            printf("no data for reading\n");
            usleep(2 * 1000);
            continue;
        }
        if (res == -1)
        {
            client_->Close();
            usleep(2 * 1000 * 1000);
            if (client_->ConnectTo(ip.c_str(), port) == -1)
            {
                printf("net work is down, try reconnecting...\n");
                continue;
            }
            res = client_->Update();
            if (res == -1)
                continue;
        }
        try_count = 0;
        std::vector< char > receive_data;
        receive_data.resize(res);
        memcpy(&receive_data[0], client_->GetData(), res);
        buf.push_back(std::move(receive_data));
        if (buf.size() > 2)
        {
            buf.pop_front();
        }
        ret = true;
        break;
    }
    return stop ? false : ret;
}

/* 离线仿真模式更新数据 */
bool CalmcarSdk::OfflineUpdate()
{
    bool ret = false;
    calmcar::proto::Frame frame;
    cv::Mat mat;
    int vector_size = 0;

    if (capture->read(mat))
    {
        in_stream.read(reinterpret_cast< char* >(&read_size), sizeof(int));
        if (max_read_size < read_size)
        {
            delete read_buf;
            read_buf = new char[read_size];
            max_read_size = read_size;
        }
        in_stream.read(read_buf, read_size);
        frame.ParseFromArray(read_buf, read_size);

        auto image = frame.mutable_raw_image();
        auto header = frame.mutable_header();
        auto image_info = header->mutable_image_info();
        image_info->set_width(mat.cols);
        image_info->set_height(mat.rows);
        image->set_data(mat.data, mat.total() * mat.elemSize());

        int total_size = frame.ByteSize();

        if (vector_size < total_size)
        {
            delete proto_buffer;
            proto_buffer = new char[total_size];
            vector_size = total_size;
        }

        frame.SerializeToArray(proto_buffer, total_size);
        std::vector< char > tmp;
        tmp.resize(total_size);
        memcpy(&tmp[0], proto_buffer, total_size);
        buf.push_back(std::move(tmp));
        if (buf.size() > 2)
        {
            buf.pop_front();
        }
        ret = true;
    }
    else
    {
        ret = false;
    }
    return stop ? false : ret;
}
}  // namespace calmcar_perception
