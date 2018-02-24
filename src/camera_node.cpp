#include <ros/ros.h>
#include <camera_aravis/camera_aravis.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_aravis");
  camera_aravis::CameraNode camera;
  camera.Start();
  ros::spin();
}
