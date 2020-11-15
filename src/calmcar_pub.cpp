#include <signal.h>
#include <chrono>
#include <cstring>
#include <iostream>
#ifdef __linux__
#include <unistd.h>
#endif

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <jpeglib.h>
#include "calmcar.pb.h"
#include "calmcar_sdk.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace std;

bool stop = false;

calmcar_perception::CalmcarSdk* interface = new calmcar_perception::CalmcarSdk();
calmcar::proto::Frame frame;
char* image_data = nullptr;

/* 处理系统信号函数 */
void catch_signal(int si)
{
    interface->Stop();
    stop = true;
}

int init(string target_ip_, int port_)
{
    int ret;
    ret = interface->Init(target_ip_, port_, calmcar_perception::VBOX);
    cout << "ONLINE mode..." << endl;
    cout << "ret=" << ret << endl;
    return ret;
}

/* JPEG格式图像解码函数
 * in_buffer:待解码图像的地址
 * size:待解码图像的大小
 * out_Mat:解码后传出的Mat结构
 * */
void Decompress_Jpeg(unsigned char* in_buffer, unsigned long size, cv::Mat& out_Mat)
{
    struct jpeg_decompress_struct jds;
    struct jpeg_error_mgr jem;
    unsigned char* decompress_buffer = (unsigned char*)in_buffer;
    unsigned long jpeg_size = size;
    jds.err = jpeg_std_error(&jem);
    jpeg_create_decompress(&jds);
    jpeg_mem_src(&jds, decompress_buffer, jpeg_size);
    jpeg_read_header(&jds, TRUE);
    jpeg_start_decompress(&jds);
    out_Mat = cv::Mat(jds.output_height, jds.output_width, CV_8UC3);
    JSAMPROW row_point;
    int status;
    while (jds.output_scanline < jds.output_height)
    {
        row_point = out_Mat.ptr() + jds.output_scanline * out_Mat.cols * 3;
        status = jpeg_read_scanlines(&jds, &row_point, 1);
    }
    usleep(10 * 1000);
    // int status=jpeg_read_scanlines(&jds,row_point,jds.output_height);
    status = jpeg_finish_decompress(&jds);
    jpeg_destroy_decompress(&jds);
    cv::cvtColor(out_Mat, out_Mat, CV_RGB2BGR);
}

cv::Mat get_image_data()
{
    uint64_t image_height;
    uint64_t image_width;
    cv::Mat show_mat;
    int pixel_format = interface->GetPixelFormat();
    frame = interface->GetData();

    image_height = interface->GetHeight();
    image_width = interface->GetWidth();
    // cout<<"interface->GetHeight():"<<image_height<<endl;
    // cout<<"interface->GetWidth():"<<image_width<<endl;
    image_height = frame.header().image_info().height();
    image_width = frame.header().image_info().width();
    // cout<<"frame.header().image_info().height():"<<image_height<<endl;
    // cout<<"frame.header().image_info().width():"<<image_width<<endl;
    // cout<<"22:"<<image_height<<","<<image_width<<endl;
    if (frame.has_raw_image())
    {
        image_data = (char*)frame.raw_image().data().data();
    }
    else
    {
        int image_size = image_height * 1.5 * image_width;
        image_data = new char[image_size];
    }

    if (pixel_format == calmcar_perception::PIXEL_FORMAT_JPEG)
    {
        unsigned char* data = (unsigned char*)frame.raw_image().data().data();
        unsigned long size = frame.raw_image().data().size();
        Decompress_Jpeg(data, size, show_mat);
    }
    else
    {
        // init show_mat
        show_mat = cv::Mat(image_height * 1.5, image_width, CV_8UC1, image_data);
        cv::cvtColor(show_mat, show_mat, CV_YUV2BGR_I420);
    }
    // cv::imshow("test", show_mat);
    // cv::waitKey(1);
    return show_mat;
}

/* 主函数 */
int main(int argc, char* argv[])
{
#ifdef __linux__
    signal(SIGINT, catch_signal);
    signal(SIGKILL, catch_signal);
    signal(SIGQUIT, catch_signal);
#endif

    ros::init(argc, argv, "kbd_cam_node");
    ros::NodeHandle nh_kbd_cam;
    ros::Publisher kbd_cam = nh_kbd_cam.advertise< sensor_msgs::Image >("/kbd_cam_node/raw_kbd_cam", 1);  // publish
    // image_transport::Publisher kbd_cam_front = image_trans.advertise("/kbd_cam_node/raw_kbd_cam_front", 1);  // publish
    // image_transport::Publisher kbd_cam_rear = image_trans.advertise("/kbd_cam_node/raw_kbd_cam_rear", 1);    // publish

    string target_ip;
    int port;
    nh_kbd_cam.param< string >("kbd_cam_node/target_ip", target_ip, "192.168.196.73");
    nh_kbd_cam.param< int >("kbd_cam_node/port", port, 9000);

    sensor_msgs::ImagePtr kbd_img;
    cv::Mat raw_image;
    int ret = init(target_ip, port);
    if (ret == -1)
        return 0;

    while (ros::ok() && stop == false)
    {
        if (interface->Update())
        {
            raw_image = get_image_data();
            kbd_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_image).toImageMsg();
            // kbd_img->header.stamp = ros::Time::now();
            // kbd_img.header.stamp = frame.timestamp();
            //cout<<"frame.timestamp():"<<frame.timestamp()<<endl;
            // ros::Time timenow(double(frame.timestamp()));
            double secNow = int(frame.timestamp()/(1e6));
            
            kbd_img->header.stamp.sec = secNow;
            kbd_img->header.stamp.nsec = double(frame.timestamp()-secNow*1e6);

            kbd_cam.publish(kbd_img);
        }
    }
    return 0;
}
