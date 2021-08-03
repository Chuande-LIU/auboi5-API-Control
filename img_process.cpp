#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include <realsense2_camera/Extrinsics.h>
#include <realsense2_camera/IMUInfo.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


using namespace realsense2_camera;
using namespace std;
#include <vector>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "image_processing");
  ros::NodeHandle nh;
  //相机参数--------------------------------------------------------------------
    rs2::pipeline pipe;
    rs2::config myconfig;
    myconfig.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    myconfig.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    //开始传送数据
    rs2::pipeline_profile myprofile = pipe.start(myconfig);

    //声明数据流
    //auto depth_stream = myprofile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = myprofile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    //rs2_intrinsics intrinDepth = depth_stream.get_intrinsics();
    rs2_intrinsics intrinColor = color_stream.get_intrinsics();
  //--------------------------------------------------------------------------------

    int depth_z=0;
    float pd_uv[2]={0,0};
    float real_pose[3]={0};
  rs2_deproject_pixel_to_point(real_pose, &intrinColor, pd_uv, depth_z);
  cout<<"x: "<<real_pose[0]<<" y: "<<real_pose[1]<<" z: "<<real_pose[2]<<endl;


}