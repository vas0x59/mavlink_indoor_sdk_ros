// #include "src/helpers.h"
#include "drone.h"
#include "tcp.h"
#include "udp.h"
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/aruco.hpp>

#include "markers_lib/src/markers.h"
#include "markers_lib/src/aruco_markers.h"
#include "markers_lib/src/solver.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>

using namespace markers_lib;
using namespace mavlink_indoor_sdk;

ArucoMarkersDetector aruco_detector;
Solver solver;

Mat cameraMatrix, distCoeffs;
bool has_camera_info = false;

// Config
string config_path = "./config.yaml";
string calibration_file = "./log.yml";
string map_url = "./map.txt";
string map_jpeg = "./map.jpg";
int map_jpeg_size = 1000;
int dictinary = 3;
int cam_id = 2;

UDP_Protocol *lw_proto;

//Camera info
static void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo,
                            cv::Mat &matrix, cv::Mat &dist)
{
    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
            matrix.at<double>(i, j) = cinfo->K[3 * i + j];

    for (unsigned int k = 0; k < cinfo->D.size(); k++)
        dist.at<double>(k) = cinfo->D[k];
}
void cinfoCallback(const sensor_msgs::CameraInfoConstPtr &cinfo)
{
    if (has_camera_info != true)
    {
        parseCameraInfo(cinfo, cameraMatrix, distCoeffs);
        has_camera_info = true;
        solver.set_camera_conf(cameraMatrix, distCoeffs);

        std::cout << "Cam OK: " << cameraMatrix << "dist: " << distCoeffs << std::endl;
    }
    // solver.se
    //  cout << "hello2";
}

struct VisionData
{
    Pose pose;
    bool success;
};

VisionData process_img(Mat img)
{
    Mat viz;
    img.copyTo(viz);
    VisionData vd;
    Mat objPoints, imgPoints;
    aruco_detector.detect(img, objPoints, imgPoints, viz);
    // aruco_detector.drawViz(viz);

    // cout << objPoints << "img: " << imgPoints << "\n";
    // Pose pose;
    vd.success = solver.solve(objPoints, imgPoints, vd.pose, viz);
    imshow("Viz", viz);
    waitKey(1);
    return vd;
}

void send_vpe(Mat frame)
{
    if (lw_proto->status == 1)
    {
        mavlink_message_t msg;
        mavlink_vision_position_estimate_t vpe;

        VisionData vd = process_img(frame);

        vpe.usec = get_time_usec();
        vpe.x = vd.pose.pose.x;
        vpe.y = vd.pose.pose.y;
        vpe.z = vd.pose.pose.z;

        vpe.roll = vd.pose.rotation.y;
        vpe.pitch = vd.pose.rotation.x;
        vpe.yaw = vd.pose.rotation.z;

        mavlink_msg_vision_position_estimate_encode(12, MAV_COMP_ID_ALL, &msg, &vpe);
        lw_proto->write_message(msg);
        usleep(10000);
    }
}

void load_config()
{
    FileStorage fs2(config_path, FileStorage::READ);
    fs2["map"] >> map_url;
    fs2["calibration_file"] >> calibration_file;

    // fs2["connection_url"] >> connection_url;

    fs2["map_jpeg"] >> map_jpeg;
    fs2["map_jpeg_size"] >> map_jpeg_size;
    fs2["dictinary"] >> dictinary;
    fs2["cam_id"] >> cam_id;
    string path = config_path.substr(0, config_path.find_last_of('/'));
    map_url = path + "/" + map_url;
    calibration_file = path + "/" + calibration_file;
    map_jpeg = path + "/" + map_jpeg;

    std::cout << "path: " << path << std::endl;
    // std::cout << "connection_url: " << connection_url << std::endl;
    fs2.release();
}

void init_marker_reg()
{

    solver.load_camera_conf(calibration_file);

    // solver.set_camera_conf(cameraMatrix, distCoeffs);

    aruco_detector.setDictionary(cv::aruco::getPredefinedDictionary(dictinary));

    aruco_detector.loadMap(map_url);
    aruco_detector.genBoard();
    Mat map_img =
        aruco_detector.drawBoard(cv::Size(map_jpeg_size, map_jpeg_size));
    imwrite(map_jpeg, map_img);
}

void imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (has_camera_info == true)
    {
        send_vpe(cv_ptr->image);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::param::get("~config_path", config_path);
     ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("image", 1, imageCb);

    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    solver.set_camera_conf(cameraMatrix, distCoeffs);
    ros::Subscriber cinfo_sub = nh.subscribe("camera_info", 1, cinfoCallback);

    load_config();
    init_marker_reg();

    lw_proto = new UDP_Protocol("udp://127.0.0.1:14510");
    lw_proto->start();

    lw_proto->stop();
}