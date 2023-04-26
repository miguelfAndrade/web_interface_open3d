#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <curl/curl.h>


// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

// using namespace std;
using namespace gazebo;


void sendPoints(std::string x, std::string y, std::string z);

std::string points_coordinates = "";

void cb(ConstLaserScanStampedPtr &_msg)
{
    gazebo::msgs::LaserScan scan = _msg->scan();
    gazebo::msgs::Pose world_data = scan.world_pose();
    gazebo::msgs::Vector3d position = world_data.position();
    
    //Laser scanner orientation from reference frame (in quaternions)
    gazebo::msgs::Quaternion laser_orientation_ = world_data.orientation();
    ignition::math::Quaternion<double> laser_orientation =  ignition::math::Quaternion(laser_orientation_.w(), laser_orientation_.x(), laser_orientation_.y(), laser_orientation_.z());
    
    //get quaternion for each ray (assuming a single vertical column of rays) @sensor frame
    std::vector<ignition::math::Quaternion<double>> ray_quat;
    ignition::math::Quaternion<double> q;
    double pitchangle = scan.angle_min();
    for(unsigned int i=0;i<scan.ranges_size();i++){
        //Set the quaternion from Euler angles; the order of operations is roll, pitch, yaw
        //The rays are attached to the joint, and its axis points along the x direction; 
        //rays rotation along the normal axis (y) corresponds to pitch angle
        q =  ignition::math::Quaternion<double>::EulerToQuaternion(0, -pitchangle,0);
//std::cout << "pitch angle = " << pitchangle << std::endl;
//std::cout << "quaternion: w=" << q.W() << " x=" << q.X() << " y=" << q.Y() << " z=" << q.Z() << std::endl;
        ray_quat.push_back(q);
        pitchangle += scan.angle_step();
    }
    
    //convert quaternions to world frame
    for(unsigned int i=0;i<ray_quat.size();i++){
        ray_quat[i] =  ray_quat[i]*laser_orientation;
    }    
    
    //std::cout << std::endl;
                
    //calculate echo positions: rotate vector in sensor frame (module equal to range)
    //std::vector<ignition::math::Vector3<double>> positions;
    // velodyne_plugin_msgs::msgs::EchoPositions posmsg;
    for(unsigned int i=0;i<ray_quat.size();i++){
        ignition::math::Vector3<double> pos(scan.ranges(i),0,0);
//std::cout << "(before rotation) echo location: x=" << pos.X() << " y=" << pos.Y() << " z=" << pos.Z() << std::endl;
//std::cout << "quaternion: w=" << ray_quat[i].W() << " x=" << ray_quat[i].X() << " y=" << ray_quat[i].Y() << " z=" << ray_quat[i].Z() << std::endl;
        pos = ray_quat[i].RotateVector(pos);
        //positions.push_back(pos);
        gazebo::msgs::Vector3d pos_;
        pos_.set_x(pos.X()+position.x());
        pos_.set_y(pos.Y()+position.y());
        pos_.set_z(pos.Z()+position.z());
        // gazebo::msgs::Vector3d* posptr = posmsg.add_positions();
        // *posptr = pos_;
//std::cout << "(after rotation)  echo location: x=" << pos.X() << " y=" << pos.Y() << " z=" << pos.Z() << std::endl;
//std::cout << "echo location: x=" << pos.X()+position.x() << " y=" << pos.Y()+position.y() << " z=" << pos.Z()+position.z() << std::endl;

        std::string st_x = std::to_string(pos.X()+position.x());
        std::string st_y = std::to_string(pos.Y()+position.y());
        std::string st_z = std::to_string(pos.Z()+position.z());
        // string cmp = "-nan";
        // if(st_x != cmp)
        // {
        //   points_coordinates += st_x + "\t" + st_y + "\t" + st_z + "\n";
        // }

        points_coordinates += st_x + "\t" + st_y + "\t" + st_z + "\n";
        sendPoints(st_x, st_y, st_z);
    }

    //std::cout << points_coordinates;
    std::ofstream pointCloud;
    pointCloud.open("../sensor_data/sensor_data.xyz"); //open is the method of ofstream
    pointCloud << points_coordinates;
    pointCloud.close();
    
    //velodyne_plugin_msgs::msgs::EchoPositions posmsg;
    //posmsg.positions = positions;
}

void sendPoints(std::string x, std::string y, std::string z) {

  CURL *curl;
  CURLcode res;
  std::string url = "http://localhost:8080/points";
  std::string data = "x=" + x + "&y=" + y + "&z=" + z;

  curl_global_init(CURL_GLOBAL_ALL);

  curl = curl_easy_init();
  if (curl)
  {
      std::string url_with_data = url + "?" + data;
      curl_easy_setopt(curl, CURLOPT_URL, url_with_data.c_str());
      res = curl_easy_perform(curl);
      if (res != CURLE_OK)
      {
          std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
      }
      curl_easy_cleanup(curl);
  }

  curl_global_cleanup();

}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

    // if(!isdigit(to_string(atoi(_argv[0]))))
    // {
    //     cout << "\nPlease enter a valid number as an argument for max_counter variable!!\n\n";
    //     return 0;
    // }

    // int max_counter = atoi(_argv[0]);

    // for (int i = 0; i < _argc; ++i) {
    //   cout << _argv[i] << endl;
    // }

    //int max_counter = 200;
    
    // Load gazebo as a client
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
    #else
    gazebo::client::setup(_argc, _argv);
    #endif

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    
    // Publish to the  velodyne topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/my_velodyne/velodyne_hdl-32/top/sensor/scan", cb);

    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif
}