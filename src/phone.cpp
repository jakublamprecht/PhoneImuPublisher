#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char **argv)
{
  int startSec;
  int startNsec;
  ros::Duration timeDiff;
  ros::Time previousTime;
  ros::Time timeNow;
  ros::Time timeStamp;
  string line = "";
  string filePath = "";
  fstream dataFile;
  
  ros::init(argc, argv, "phone_publisher");
  
  ros::NodeHandle n;
  ros::Publisher phonePub = n.advertise<sensor_msgs::Imu>("/phone/imu", 1000);
  ros::Rate loop_rate(100);

  previousTime.sec = 0;
  previousTime.nsec = 0;
  
  n.param<std::string>("/phone_publisher/dataFilePath", filePath, "dummypath.txt");
  n.getParam("/phone_publisher/secs", startSec);
  n.getParam("/phone_publisher/nsecs", startNsec);
  
  timeStamp.sec = startSec;
  timeStamp.nsec = startNsec;
  
  dataFile.open(filePath.c_str(), ios::in);

  if (dataFile.is_open()) {
    while (ros::ok()) {
      if (dataFile.eof()) {
        ROS_INFO("Reached End of File");
        break;
      }
      
      float qw, qx, qy, qz, qt;
      float ax, ay, az, at;
      float gx, gy, gz, gt;
      float mx, my, mz, mt;

      char tmp;
      dataFile >> tmp >> qw >> tmp >> qx >> tmp >> qy >> tmp >> qz >> tmp >> qt >> tmp;
      dataFile >> tmp >> ax >> tmp >> ay >> tmp >> az >> tmp >> at >> tmp;
      dataFile >> tmp >> gx >> tmp >> gy >> tmp >> gz >> tmp >> gt >> tmp;
      dataFile >> tmp >> mx >> tmp >> my >> tmp >> mz >> tmp >> mt >> tmp;
          
      ROS_INFO("Values Q: %f %f %f %f %f", qw, qx, qy, qz, qt);
      ROS_INFO("Values A: %f %f %f %f", ax, ay, az, at);
      ROS_INFO("Values G: %f %f %f %f", gx, gy, gz, gt);
      ROS_INFO("Values M: %f %f %f %f", mx, my, mz, mt);
      
      timeNow = ros::Time::now();
      timeDiff = timeNow - previousTime;
      timeStamp = timeStamp + timeDiff;

      sensor_msgs::Imu imuMsg;

      imuMsg.header.frame_id = "imu_phone";
      imuMsg.header.stamp = timeStamp;

      imuMsg.orientation.w = qw;
      imuMsg.orientation.x = qx;
      imuMsg.orientation.y = qy;
      imuMsg.orientation.z = qz;

      // No data from phone

      imuMsg.angular_velocity.x = gx;
      imuMsg.angular_velocity.y = gy;
      imuMsg.angular_velocity.z = gz;

      imuMsg.linear_acceleration.x = ax;
      imuMsg.linear_acceleration.y = ay;
      imuMsg.linear_acceleration.z = az;

      phonePub.publish(imuMsg);

      previousTime = timeNow;
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  else {
    ROS_INFO("Could not open file");
  }

  return 0;
}
