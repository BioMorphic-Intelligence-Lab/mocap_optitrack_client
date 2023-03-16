#ifndef MOCAPPUBLISHER_H
#define MOCAPPUBLISHER_H

#include <vector>
#include <NatNetTypes.h>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_ros_com/frame_transforms.h"


using namespace std;
using namespace std::chrono;
class MoCapPublisher: public rclcpp::Node
{
private:
    //Attributes
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _px4_publisher;

    rclcpp::TimerBase::SharedPtr timer_;
    high_resolution_clock::time_point t_start;

    //Rigid body indeces
    int am_rigid_body_idx, wall_rigid_body_idx;

    //Publishing topic names
    std::string _pub_topic, _px4_topic;

    //Transformation Functions
    geometry_msgs::msg::Pose _mocap2ros(sRigidBodyData body);
    px4_msgs::msg::VehicleOdometry _ros2px4(geometry_msgs::msg::PoseStamped pose);

    //Callback function
    void _timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);

    Eigen::Matrix3d _rot_x(double theta);
    Eigen::Matrix3d _rot_y(double theta);
    Eigen::Matrix3d _rot_z(double theta);
    

public:
    // Definition of the construtors
    MoCapPublisher();

    // Send methods
    void sendRigidBodyMessage(sRigidBodyData* bodies_ptr, int nRigidBodies);
 
    // Getters
    std::string getServerAddress();
    int getConnectionType();
    std::string getMulticastAddress();
    uint16_t getServerCommandPort();
    uint16_t getServerDataPort();
    
    // Setters

    // Time offset
    std::atomic<uint64_t> _timestamp_remote;
    std::chrono::time_point<std::chrono::steady_clock> _timestamp_local;
};
 
#endif