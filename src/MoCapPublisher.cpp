// RoS2 Node that handles the connection with the NatNet server (Motive)
#include <MoCapPublisher.h>

// Include standard libraries
#include <stdio.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
// Include the MoCap NatNet client
#include <MoCapNatNetClient.h>
#include "common/common.hpp"

using namespace std;
using namespace std::chrono_literals;
using namespace std::chrono;

bool cmpRigidBodyId(sRigidBodyData body_a, sRigidBodyData body_b)
{
  return body_a.ID < body_b.ID;
}

MoCapPublisher::MoCapPublisher(): Node("natnet_client")
{

  //Declare the ROS2 parameters used by the NatNet Client
  this->declare_parameter<std::string>("server_address", "10.125.37.2");
  this->declare_parameter<int>("connection_type", 0);
  this->declare_parameter<std::string>("multi_cast_address", "239.255.42.99");
  this->declare_parameter<uint16_t>("server_command_port", 1510);
  this->declare_parameter<uint16_t>("server_data_port", 1511);
  this->declare_parameter<std::string>("pub_topic", "optitrack_pose");
  this->declare_parameter<std::string>("ee_topic", "ee_pose");
  this->declare_parameter<std::string>("wall_topic", "wall_pose");
  // Needs to be visual odometry instead of mocap odometry sice EKF2 (in px4) only subscribes to that topic
  this->declare_parameter<std::string>("px4_topic", "/fmu/in/vehicle_visual_odometry");
  this->declare_parameter<uint16_t>("am_rigid_body_idx", -1);
  this->declare_parameter<uint16_t>("ee_rigid_body_idx", -1);
  this->declare_parameter<uint16_t>("wall_rigid_body_idx", -1);

  this->am_rigid_body_idx = this->get_parameter("am_rigid_body_idx").as_int();
  this->ee_rigid_body_idx = this->get_parameter("ee_rigid_body_idx").as_int();
  this->wall_rigid_body_idx = this->get_parameter("wall_rigid_body_idx").as_int();
  //
  //Create the publishers
  this->_pub_topic = this->get_parameter("pub_topic").as_string();
  this->_ee_topic = this->get_parameter("ee_topic").as_string();
  this->_wall_topic = this->get_parameter("wall_topic").as_string();

  this->_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->_pub_topic.c_str(), 10);
  this->_ee_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->_ee_topic.c_str(), 10);
  this->_wall_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->_wall_topic.c_str(), 10);

  this->_px4_topic = this->get_parameter("px4_topic").as_string();
  this->_px4_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>(this->_px4_topic.c_str(), 10);
  //
  //Create Subscriptions
  this->_timesync_sub = this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", rclcpp::SensorDataQoS(), std::bind(&MoCapPublisher::_timesync_callback, this, std::placeholders::_1));

  //Get the current time for the timestamp of the messages
  this->t_start = high_resolution_clock::now();//get the current time
  //
  //Just for testing purposes send make messages every 500ms
  //this->timer_ = this->create_wall_timer(500ms, std::bind(&MoCapPublisher::sendFakeMessage, this));
  //
  //Log info about creation
  RCLCPP_INFO(this->get_logger(), "Created MoCap publisher node.\n");

  //TO REMOVE
  std::string address_;
  this->get_parameter("server_address", address_);
  RCLCPP_INFO(this->get_logger(),address_.c_str());
}


void MoCapPublisher::_timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
{
  this->_timestamp_local = std::chrono::steady_clock::now();
  this->_timestamp_remote.store(msg->timestamp);
}

// Method that send over the ROS network the data of a rigid body
void MoCapPublisher::sendRigidBodyMessage(sRigidBodyData* bodies_ptr, int nRigidBodies)
{
  sRigidBodyData body;
  sRigidBodyData ee;
  sRigidBodyData wall;
  
  for(int i=0; i < nRigidBodies; i++) 
  {
    if(bodies_ptr[i].ID == this->am_rigid_body_idx) body = bodies_ptr[i];
    else if(bodies_ptr[i].ID == this->ee_rigid_body_idx) ee = bodies_ptr[i];
    else if(bodies_ptr[i].ID == this->wall_rigid_body_idx) wall = bodies_ptr[i];
  }

  rclcpp::Time now = this->now();
  if(this->am_rigid_body_idx >= 0)
  {
    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header.stamp = this->now();
    base_pose.header.frame_id = "world";

    RCLCPP_DEBUG(this->get_logger(), "MOCAP q = [w: %f, x: %f, y: %f, z: %f]",
                                  body.qw, body.qx, body.qy, body.qz);


    base_pose.pose = this->_mocap2ros(body);
    this->_pose_publisher->publish(base_pose);


    RCLCPP_DEBUG(this->get_logger(), "ROS q = [w: %f, x: %f, y: %f, z: %f]",
                                  base_pose.pose.orientation.w,
                                  base_pose.pose.orientation.x,
                                  base_pose.pose.orientation.y,
                                  base_pose.pose.orientation.z);

    if(!this->_px4_topic.empty())
    {
      px4_msgs::msg::VehicleOdometry px4_pose = this->_ros2px4(base_pose);

      RCLCPP_DEBUG(this->get_logger(), "PX4 q = [w: %f, x: %f, y: %f, z: %f]",
                                  px4_pose.q[0],
                                  px4_pose.q[1],
                                  px4_pose.q[2],
                                  px4_pose.q[3]);

      this->_px4_publisher->publish(px4_pose);
    }
  }


  if(this->ee_rigid_body_idx >= 0)
  {

    geometry_msgs::msg::PoseStamped ee_pose;
    
    ee_pose.header.stamp = this->now();
    ee_pose.header.frame_id = "world";
    ee_pose.pose = this->_mocap2ros(ee);
    this->_ee_publisher->publish(ee_pose);
  }

  if(this->wall_rigid_body_idx >= 0)
  {
    geometry_msgs::msg::PoseStamped wall_pose;
    
    wall_pose.header.stamp = this->now();
    wall_pose.header.frame_id = "world";
    wall_pose.pose = this->_mocap2ros(wall);
    this->_wall_publisher->publish(wall_pose);
  }
}


px4_msgs::msg::VehicleOdometry MoCapPublisher::_ros2px4(geometry_msgs::msg::PoseStamped pose)
{
  px4_msgs::msg::VehicleOdometry px4_pose;
  auto now = std::chrono::steady_clock::now();

  px4_pose.timestamp = this->_timestamp_remote.load() + std::chrono::round<std::chrono::microseconds>(now - this->_timestamp_local).count();
  px4_pose.timestamp_sample = px4_pose.timestamp;
  px4_pose.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;


  Eigen::Vector3d enu_p(pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z);
  Eigen::Vector3d ned_p = common::enu_2_ned(enu_p);

  Eigen::Quaterniond enu_q(pose.pose.orientation.w,
                           pose.pose.orientation.x,
                           pose.pose.orientation.y,
                           pose.pose.orientation.z);
  Eigen::Quaterniond ned_q = common::enu_2_ned(enu_q);

  
  px4_pose.position = {(float)ned_p.x(), (float)ned_p.y(), (float)ned_p.z()};
  px4_pose.q = {(float)ned_q.x(), (float)ned_q.y(), (float)ned_q.z(), (float)ned_q.w()};

  return px4_pose;

}

geometry_msgs::msg::Pose MoCapPublisher::_mocap2ros(sRigidBodyData body)
{
  geometry_msgs::msg::Pose pose;

  Eigen::Matrix3d transformation = common::rot_z(M_PI);
  Eigen::Quaterniond q = Eigen::Quaterniond(transformation);

  Eigen::Vector3d position;
  position << body.x, body.y, body.z;
  position = transformation*position;

  Eigen::Quaterniond orientation(body.qw, body.qx, body.qy, body.qz);
  orientation =  q * orientation;

  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
    
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();

  return pose;
}


std::string MoCapPublisher::getServerAddress()
{
  std::string addr_;
  this->get_parameter("server_address", addr_);
  return addr_;
}

int MoCapPublisher::getConnectionType()
{
  int type_ = 0;
  this->get_parameter("connection_type", type_);
  return type_;
}

std::string MoCapPublisher::getMulticastAddress()
{
  std::string addr_;
  this->get_parameter("multi_cast_address", addr_);
  return addr_;
}

uint16_t MoCapPublisher::getServerCommandPort()
{
  uint16_t port_;
  this->get_parameter("server_command_port", port_);
  return port_;
}

uint16_t MoCapPublisher::getServerDataPort()
{
  uint16_t port_;
  this->get_parameter("server_data_port", port_);
  return port_;
}


// Main
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  // Initialize ROS2
  rclcpp::init(argc, argv);

  //Create the ROS2 publisher
  auto mocapPub = std::make_shared<MoCapPublisher>();
  //Create the MoCapNatNetClient
  MoCapNatNetClient* c = new MoCapNatNetClient(mocapPub.get());
  // Try to connect the client 
  int retCode = c->connect();
  if (retCode != 0)
  {
    return retCode;
  }
  // Ready to receive marker stream
  rclcpp::spin(mocapPub);
  // Delete all the objects created
  delete c;
  rclcpp::shutdown();//delete the ROS2 nodes
  return 0;
}
