#ifndef HOLO_SWARM_PKG__SINGLE_UAV_HPP_
#define HOLO_SWARM_PKG__SINGLE_UAV_HPP_

#include <atomic>
#include <vector>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Messages
#include <std_srvs/srv/trigger.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/reference_array.hpp>
#include <mrs_msgs/msg/gps_data.hpp>
#include <mrs_msgs/srv/path_srv.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/transformer.h>

namespace single_uav
{
  class SingleUAV : public rclcpp::Node
  {
  public:
    SingleUAV(rclcpp::NodeOptions options);

  private:
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> is_initialized_ = false;

    // Timers //{
    rclcpp::TimerBase::SharedPtr timer_initialization_;
    void InitializationCallback();
    rclcpp::TimerBase::SharedPtr timer_publishing_;
    rclcpp::TimerBase::SharedPtr timer_tf_;
    rclcpp::CallbackGroup::SharedPtr callbackGroup_;
    //}
    
    // UAV --------------------------//{
    // Subscription
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_odom_;
    void SubOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    nav_msgs::msg::Odometry latest_Odom_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::ConstSharedPtr sub_poses_;
    void SubPosesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg);
    geometry_msgs::msg::PoseArray latest_Poses_; 

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::ConstSharedPtr sub_bounderies_;
    void SubBoundariesCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr &msg);
    visualization_msgs::msg::MarkerArray latest_Boundaries_;

    // Clients 
    rclcpp::Client<mrs_msgs::srv::ReferenceStampedSrv>::SharedPtr cli_sendRef_;
    bool SendRefence(mrs_msgs::msg::ReferenceStamped);
    rclcpp::Client<mrs_msgs::srv::PathSrv>::SharedPtr cli_sendTrajectory_;
    bool SendTrajectory(mrs_msgs::msg::Path); 
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_stopTrajectoryTracking_; 
    bool StopTrajectoryTracking();
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_startTrajectoryTracking_;
    bool StartTrajectoryTracking();
  
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_takeOff_;
    bool TakeOff();
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_land_;
    bool Land();
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_home_;
    bool Home();
    //}
    
    // HOLO_TABLE -----------------------//{
    //Subscribers
    rclcpp::Subscription<mrs_msgs::msg::ReferenceStamped>::ConstSharedPtr sub_flyToWaypoint_;
    void SubFlyToWaypointCallback(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr &msg);   
    rclcpp::Subscription<mrs_msgs::msg::ReferenceArray>::ConstSharedPtr sub_flyThroughWaypoints_;
    void SubFlyThroughWaypoints(const mrs_msgs::msg::ReferenceArray::ConstSharedPtr &msg);  
    // Publishers 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_traj_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_boundaries_;
    void HoloPublishCallback();
    //Services
    rclcpp::Service<std_srvs::srv::Trigger>::ConstSharedPtr ser_land_;
    void LandCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>);
    rclcpp::Service<std_srvs::srv::Trigger>::ConstSharedPtr ser_takeoff_;
    void TakeOffCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>);
    rclcpp::Service<std_srvs::srv::Trigger>::ConstSharedPtr ser_home_;
    void HomeCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>);
    //}
    
    // TF Tree utilities //{
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<mrs_lib::Transformer> transformer_; 
    //}
    
    // Miscel..
    bool IsInitialized(std::string functionName);
    mrs_msgs::msg::Path GetPathFromWaypoints(const mrs_msgs::msg::ReferenceArray wayp);
    // Params 
    mrs_msgs::msg::ReferenceStamped ref_;
  };

}

#endif
