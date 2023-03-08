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
  this->declare_parameter<std::string>("px4_topic", "/fmu/in/mocap_vehicle_odometry");
  this->declare_parameter<uint16_t>("am_rigid_body_idx", -1);
  this->declare_parameter<uint16_t>("wall_rigid_body_idx", -1);

  this->am_rigid_body_idx = this->get_parameter("am_rigid_body_idx").as_int();
  this->wall_rigid_body_idx = this->get_parameter("wall_rigid_body_idx").as_int();
  //
  //Create the publishers
  this->_pub_topic = this->get_parameter("pub_topic").as_string();
  this->_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->_pub_topic.c_str(), 10);

  this->_px4_topic = this->get_parameter("px4_topic").as_string();
  this->_px4_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>(this->_px4_topic.c_str(), 10);
  //
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

// Method that send over the ROS network the data of a rigid body
void MoCapPublisher::sendRigidBodyMessage(sRigidBodyData* bodies_ptr, int nRigidBodies)
{
  sRigidBodyData body;
  for(int i=0; i < nRigidBodies; i++) 
  {
    if(bodies_ptr[i].ID == this->am_rigid_body_idx) body = bodies_ptr[i];
  }

  rclcpp::Time now = this->now();

  //Instanciate variables
  if(this->am_rigid_body_idx >= 0)
  {
    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header.stamp = this->now();
    base_pose.header.frame_id = "world";

    base_pose.pose = this->_mocap2ros(body);
    this->_pose_publisher->publish(base_pose);

    if(!this->_px4_topic.empty())
    {
      px4_msgs::msg::VehicleOdometry px4_pose = this->_ros2px4(base_pose);
      this->_px4_publisher->publish(px4_pose);
    }
  }


  if(this->wall_rigid_body_idx >= 0)
  {
    geometry_msgs::msg::PoseStamped wall_pose;
    //TODO
  }
}


px4_msgs::msg::VehicleOdometry MoCapPublisher::_ros2px4(geometry_msgs::msg::PoseStamped pose)
{
  px4_msgs::msg::VehicleOdometry px4_pose;
  px4_pose.timestamp = (uint64_t) (pose.header.stamp.sec * 1000000 
                                 + pose.header.stamp.nanosec * 0.001);
  px4_pose.pose_frame = px4_pose.POSE_FRAME_NED;

  Eigen::Vector3d position = Eigen::Vector3d(pose.pose.position.x,
                                             pose.pose.position.y,
                                             pose.pose.position.z);
  position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(position);   
  px4_pose.position = {position.x(),
                       position.y(),
                       position.z()};
  
  Eigen::Quaterniond orientation = Eigen::Quaterniond(pose.pose.orientation.w,
                                                      pose.pose.orientation.x,
                                                      pose.pose.orientation.y,
                                                      pose.pose.orientation.z);
  orientation = px4_ros_com::frame_transforms::ned_to_enu_orientation(
                px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(orientation));  
  px4_pose.q = {orientation.x(),
                orientation.y(),
                orientation.z(),
                orientation.w()};

  return px4_pose;

}

geometry_msgs::msg::Pose MoCapPublisher::_mocap2ros(sRigidBodyData body)
{
  geometry_msgs::msg::Pose pose;
  //TODO this depends on how optitrack is set up
  Eigen::Matrix3d transformation = this->_rot_x(-M_PI_2) * this->_rot_z(M_PI);

  Eigen::Vector3d position;
  position << body.x, body.y, body.z;
  position = transformation*position;

  Eigen::Quaterniond orientation(body.qw, body.qx, body.qy, body.qz);
  Eigen::Quaterniond transformation_q = Eigen::Quaterniond(transformation); 
  orientation =  transformation_q * orientation * transformation_q.conjugate();

  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
    
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();

  return pose;
}


Eigen::Matrix3d MoCapPublisher::_rot_x(double theta)
{
  double sT = sin(theta), cT = cos(theta);
  Eigen::Matrix3d rot;
  rot << 1, 0, 0,
         0, cT, -sT,
         0, sT, cT;
  return rot;
}

Eigen::Matrix3d MoCapPublisher::_rot_y(double theta)
{

  double sT = sin(theta), cT = cos(theta);
  Eigen::Matrix3d rot;
  rot << cT, 0, sT,
         0, 1, 0,
         -sT, 0, cT;
  return rot;
}
Eigen::Matrix3d MoCapPublisher::_rot_z(double theta)
{
  double sT = sin(theta), cT = cos(theta);
  Eigen::Matrix3d rot;
  rot << cT, -sT, 0,
         sT, cT, 0,
         0, 0, 1;
  return rot;
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
