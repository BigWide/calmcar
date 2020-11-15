#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include "calmcar.pb.h"
#include "tcp_client.h"

namespace calmcar_perception
{
typedef calmcar::proto::Frame ProtobufFrame;
typedef std::pair< const char*, int > FramePair;
typedef std::vector< char > RawFrameData;

enum LineColor
{
    ColorWhite = 1,
    ColorYellow = 2,
};

enum LineType
{
    TypeSolid = 1,
    TypeDashed = 2,
};
enum
{
    ONLINE,
    OFFLINE
};

class CalmcarSdk
{
  public:
    explicit CalmcarSdk();
    virtual ~CalmcarSdk();

    bool Init(std::string, int port = 0, int f = FCM);
    bool InitOnline(std::string, int port = 0);
    bool InitOffline(std::string);
    ProtobufFrame GetData();
    uint32_t GetWidth() { return width_; }
    uint32_t GetHeight() { return height_; }
    int GetPixelFormat() { return format_; }
    bool Update();
    void Stop() { stop = true; }

  private:
    TcpClient* client_;
    std::thread thread_;
    bool OnlineUpdate();
    bool OfflineUpdate();
    std::string ip;
    std::deque< RawFrameData > buf;
    int port;
    bool stop;
    uint32_t width_;
    uint32_t height_;
    uint32_t format_;
    std::mutex mutex_;
    std::condition_variable cv;
    std::string video_file_name_;
    std::string csd_file_name_;
    cv::VideoCapture* capture;
    std::fstream in_stream;
    char* proto_buffer = nullptr;
    char* read_buf = nullptr;
    int max_read_size = 0;
    int read_size = 0;
    int mode;
    int func_;
    enum
    {
        ONLINE,
        OFFLINE,
    };
};
}  // namespace calmcar_perception

#endif
