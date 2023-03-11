#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include "echo_positions.pb.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include <curl/curl.h>

std::string points_coordinates = "";

CURL *curl;
CURLcode res;
std::string url = "http://localhost:8080/points";

void sendPoints(std::string x, std::string y, std::string z);

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      std::cout << "Velodyne plugin loaded\n";
        
      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[0];

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Default to zero velocity
      double velocity = 0;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->SetVelocity(velocity);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the velocity command topic, and register a callback
      this->sub1 = this->node->Subscribe(topicName,
         &VelodynePlugin::OnMsg, this);
         
         
      // Publish laser intersection positions topic
      this->pub = this->node->Advertise<velodyne_plugin_msgs::msgs::EchoPositions>("~/velodyne_hdl-32/echo_positions");   
         
      // Subscribe to the laser scan topic and register callback
      this->sub2 = this->node->Subscribe("~/velodyne_hdl-32/top/sensor/scan",
         &VelodynePlugin::OnRanges, this);  
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// \brief Handle incoming velocity command
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

    /// \brief Handle incoming range measurements
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnRanges(ConstLaserScanStampedPtr &_msg)
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
        velodyne_plugin_msgs::msgs::EchoPositions posmsg;
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
            gazebo::msgs::Vector3d* posptr = posmsg.add_positions();
            *posptr = pos_;
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
        pub->Publish(posmsg);
    }



    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief Velocity cmd subscriber
    private: transport::SubscriberPtr sub1;
    
    /// \brief Lidar ranges subscriber
    private: transport::SubscriberPtr sub2;

    /// \brief Lidar echo positions publisher
    private: transport::PublisherPtr pub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif


void sendPoints(std::string x, std::string y, std::string z) {
  // std::cout << x << " " << y << " " << z << " " << std::endl;

  // CURL *curl;
  // CURLcode res;
  // std::string url = "https://example.com/api";
  std::string data = "x=" << x << "&y=" << y << "&z=" << z;

  curl_global_init(CURL_GLOBAL_ALL);

  curl = curl_easy_init();
  if (curl)
  {
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
      res = curl_easy_perform(curl);
      if (res != CURLE_OK)
      {
          std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
      }
      curl_easy_cleanup(curl);
  }

  curl_global_cleanup();

}