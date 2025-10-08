#include "holo_swarm/single_uav.hpp"

//TODO: Better takeoff ?
//TODO: Multiple UAVs
  // add into multiple_uav from controller or /use another tracker/map planning node 
  //  -> transform into controll commands

namespace single_uav
{
  //Initialization //{
  SingleUAV::SingleUAV(rclcpp::NodeOptions options)
  : Node("single_uav_node", options)
  {
    timer_initialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&SingleUAV::InitializationCallback, this));
  }
  //}
  //InitializationCallback //{
  void SingleUAV::InitializationCallback(){
    node_ = shared_from_this();

    callbackGroup_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callbackGroup_;

    //UAV --------------------------
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/get_odom", 9, std::bind(&SingleUAV::SubOdomCallback, this, std::placeholders::_1), sub_opt);
    sub_poses_ = create_subscription<geometry_msgs::msg::PoseArray>("/get_poses", 9, std::bind(&SingleUAV::SubPosesCallback, this, std::placeholders::_1), sub_opt);
    sub_bounderies_ = create_subscription<visualization_msgs::msg::MarkerArray>("/get_boundaries", 9, std::bind(&SingleUAV::SubBoundariesCallback, this, std::placeholders::_1), sub_opt);  

    cli_sendRef_ = create_client<mrs_msgs::srv::ReferenceStampedSrv>("/set_reference");
    cli_sendTrajectory_ = create_client<mrs_msgs::srv::PathSrv>("/set_trajectory");
    cli_stopTrajectoryTracking_ = create_client<std_srvs::srv::Trigger>("/stop_trajectory_tracking");
    cli_startTrajectoryTracking_ = create_client<std_srvs::srv::Trigger>("/start_trajectory_tracking");
    cli_takeOff_ = create_client<std_srvs::srv::Trigger>("/set_takeoff");
    cli_land_ = create_client<std_srvs::srv::Trigger>("/set_land");
    cli_home_ = create_client<std_srvs::srv::Trigger>("/set_home");

    //HOLO_TABLE ------------------
    sub_flyToWaypoint_ = create_subscription<mrs_msgs::msg::ReferenceStamped>("/fly_to_waypoint", 9, std::bind(&SingleUAV::SubFlyToWaypointCallback, this, std::placeholders::_1), sub_opt);
    sub_flyThroughWaypoints_ = create_subscription<mrs_msgs::msg::ReferenceArray>("/fly_through_waypoints", 9, std::bind(&SingleUAV::SubFlyThroughWaypoints, this, std::placeholders::_1), sub_opt);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 1); 
    pub_traj_ = create_publisher<geometry_msgs::msg::PoseArray>("/current_trajectory", 1);
    pub_boundaries_ = create_publisher<visualization_msgs::msg::MarkerArray>("/boundary", 1);

    ser_land_ = create_service<std_srvs::srv::Trigger>("/land", std::bind(&SingleUAV::LandCallback, this, std::placeholders::_1, std::placeholders::_2));
    ser_takeoff_ = create_service<std_srvs::srv::Trigger>("/takeoff", std::bind(&SingleUAV::TakeOffCallback, this, std::placeholders::_1, std::placeholders::_2));
    ser_home_ = create_service<std_srvs::srv::Trigger>("/home", std::bind(&SingleUAV::HomeCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    timer_publishing_ = create_wall_timer(std::chrono::duration<double>(1.0/100.0), std::bind(&SingleUAV::HoloPublishCallback, this));
    is_initialized_ = true;

    //TF -------------------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
    transformer_->retryLookupNewest(true);

    timer_initialization_ -> cancel();
    RCLCPP_INFO(node_->get_logger(), "Node initialized");
  }
  //}

  // Publish callback //{
  void SingleUAV::HoloPublishCallback(){
    if(IsInitialized(__func__)){
      pub_odom_->publish(latest_Odom_);
      pub_traj_->publish(latest_Poses_);
      pub_boundaries_->publish(latest_Boundaries_);
    }
  }
  //}

  //UAV --------------------------------------------- //{
  //Subscribers callbacks //{
  void SingleUAV::SubOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg){
    if(IsInitialized(__func__)){
      try{
       geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(uav_name + "/utm_origin", msg->header.frame_id, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;

        tf2::doTransform(pose_in, pose_out, transformStamped);

        nav_msgs::msg::Odometry odom_out = *msg;
        odom_out.header.stamp = this->now();
        odom_out.header.frame_id = uav_name + "/utm_origin";
        odom_out.pose.pose = pose_out.pose;

        latest_Odom_ = odom_out;
      }
    catch (const tf2::TransformException& ex ){
      RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", ex.what());
      }
    }
  }

  void SingleUAV::SubPosesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg){
    if(IsInitialized(__func__)){
      latest_Poses_ = *msg;
    }
  }
  
  void SingleUAV::SubBoundariesCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr & msg){
    if(IsInitialized(__func__))
    {
      try{
        if(msg->markers.size() > 0){
          geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(uav_name + "/utm_origin", uav_name + "/world_origin", tf2::TimePointZero);
  
          visualization_msgs::msg::MarkerArray markerArray = *msg;

          for(size_t i = 0; i < markerArray.markers.size(); ++i){
            geometry_msgs::msg::PoseStamped pose_in, pose_out;
            pose_in.header = markerArray.markers[i].header;
            pose_in.pose = markerArray.markers[i].pose;

            tf2::doTransform(pose_in, pose_out, transformStamped);

            std_msgs::msg::Header header = markerArray.markers[i].header;
            header.frame_id = uav_name + "/utm_origin";  
            markerArray.markers[i].header = header;
            markerArray.markers[i].pose = pose_out.pose;
          }

          latest_Boundaries_ = markerArray; 
        }

      }
      catch (const tf2::TransformException& ex){
        RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
      }
    }
  }

  //} 
  //Client calls //{ 
  bool SingleUAV::SendRefence(mrs_msgs::msg::ReferenceStamped ref)
  {
    if(!IsInitialized(__func__))
    {
      return false;
    }

    auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    request->reference = ref.reference;
    request->header = ref.header;

    if(request->header.frame_id == uav_name + "/utm_origin"){
      try{
       geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(uav_name + "/world_origin", uav_name + "/utm_origin", tf2::TimePointZero);
  
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = request->header;
        pose_in.pose.position = request->reference.position;

        tf2::doTransform(pose_in, pose_out, transformStamped);

        request->header.stamp = this->now();
        request->header.frame_id = uav_name + "/world_origin";
        request->reference.position = pose_out.pose.position;
        RCLCPP_INFO(get_logger(), "Sended reference transformed");
      }
    catch (const tf2::TransformException& ex ){
      RCLCPP_WARN(node_->get_logger(), "Transform failed in setting reference: %s", ex.what());
      }

      RCLCPP_INFO_STREAM(get_logger(), "Successful transform Pos: X: " << request->reference.position.x << " Y: " << request->reference.position.y << " Z: " << request->reference.position.z);
    }
    

    if(cli_sendRef_->service_is_ready())
    {
      cli_sendRef_->async_send_request(request, [&](const rclcpp::Client<mrs_msgs::srv::ReferenceStampedSrv>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          RCLCPP_INFO(node_->get_logger(), "Reference set success");
          return true;
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Reference set response error.");
          return false;
        }

      });   
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Service SendReference is not ready.");
      return false;
    }
  }
  bool SingleUAV::SendTrajectory(mrs_msgs::msg::Path path)
  {
    if(!IsInitialized(__func__))
    {
      return false;
    }

    auto request = std::make_shared<mrs_msgs::srv::PathSrv::Request>();
    request->path = path;

    if(cli_sendTrajectory_->service_is_ready())
    {
      cli_sendTrajectory_->async_send_request(request, [&](const rclcpp::Client<mrs_msgs::srv::PathSrv>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          RCLCPP_INFO(node_->get_logger(), "Trajectory set success");
          return true;
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Trajectory set response error.");
          return false;
        }

      });   
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Service TrajectorySet is not ready.");
      return false;
    }
  } 
  bool SingleUAV::StopTrajectoryTracking()
  {
    if(!IsInitialized(__func__))
    {
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    if(cli_stopTrajectoryTracking_->service_is_ready())
    {
      cli_stopTrajectoryTracking_->async_send_request(request, [&](const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          RCLCPP_INFO(node_->get_logger(), "Trajectory tracking stop success");
          return true;
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Trajectory tracking message error.");
          return false;
        }

      });   
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Service TrajectoryStop is not ready.");
      return false;
    }
  }
  bool SingleUAV::StartTrajectoryTracking()
  {
    if(!IsInitialized(__func__))
    {
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    if(cli_startTrajectoryTracking_->service_is_ready())
    {
      cli_startTrajectoryTracking_->async_send_request(request, [&](const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          RCLCPP_INFO(node_->get_logger(), "Trajectory tracking start success");
          return true;
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Trajectory tracking start message error.");
          return false;
        }

      });   
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Service TrajectoryStart is not ready.");
      return false;
    }
  }
  bool SingleUAV::TakeOff()
  {
    if(!IsInitialized(__func__))
    {
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    if(cli_takeOff_->service_is_ready())
    {
      cli_takeOff_->async_send_request(request, [&](const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          RCLCPP_INFO(node_->get_logger(), "Takeoff send success");
          return true;
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Takeoff send error.");
          return false;
        }

      });   
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Service Takeoff is not ready.");
      return false;
    };
  }
  bool SingleUAV::Land()
  {
    if(!IsInitialized(__func__))
    {
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    if(cli_land_->service_is_ready())
    {
      cli_land_->async_send_request(request, [&](const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          RCLCPP_INFO(node_->get_logger(), "Land send success");
          return true;
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Land send error.");
          return false;
        }

      });   
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Service Land is not ready.");
      return false;
    };
  }
  bool SingleUAV::Home()
  {
    if(!IsInitialized(__func__))
    {
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    if(cli_home_->service_is_ready())
    {
      cli_home_->async_send_request(request, [&](const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          RCLCPP_INFO(node_->get_logger(), "Home send success");
          return true;
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Home send error.");
          return false;
        }
      });   
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Service Home is not ready.");
      return false;
    };
  }
  //}

  //HOLO-TABLE ----------------------------------------- //{
  //Subscriber callbacks //{
  void SingleUAV::SubFlyToWaypointCallback(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr &msg)
  {
    if(!IsInitialized(__func__))
      return;
    SendRefence(*msg);
  } 
  void SingleUAV::SubFlyThroughWaypoints(const mrs_msgs::msg::ReferenceArray::ConstSharedPtr &msg)
  {
    if(!IsInitialized(__func__))
      return;

    mrs_msgs::msg::Path path = GetPathFromWaypoints(*msg);
    RCLCPP_INFO(node_->get_logger(), "Before sending trajectory"); 
    SendTrajectory(path);  
    RCLCPP_INFO(node_->get_logger(), "After sending trajectory"); 
  }
  //}

  //Server callbacks //{ 
  void SingleUAV::LandCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>)
  {
    if(IsInitialized(__func__))
      Land();
  }
  void SingleUAV::TakeOffCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>)
  {
    //TODO: Test this better so I have working setup -> look into simulation
    if(IsInitialized(__func__))
      TakeOff();
  }
  void SingleUAV::HomeCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>)
  {
    if(IsInitialized(__func__))
      Home();
  }
  //}
  //}

  //Miscel.... //{
  bool SingleUAV::IsInitialized(std::string functionName){
      if (!is_initialized_){
        RCLCPP_INFO_STREAM(node_->get_logger(), "Node not initialized" << "| Call from: |" << functionName);
        return false; 
      }
      return true;
  }

  
  mrs_msgs::msg::Path SingleUAV::GetPathFromWaypoints(const mrs_msgs::msg::ReferenceArray wayp)
  {
    mrs_msgs::msg::Path path;
    std_msgs::msg::Header header;

    header.frame_id = "";
    path.use_heading = false;
    path.fly_now = true;
    path.stop_at_waypoints = false;
    path.loop = false;
    path.override_heading_atan2 = true;

    // --- Check and transform each waypoint if in UTM frame ---
    for (auto point : wayp.array)
    {
      std::string frame_id = wayp.header.frame_id;

      if (frame_id == uav_name +"/utm_origin")
      {
        try
        {
          geometry_msgs::msg::TransformStamped transformStamped =
              tf_buffer_->lookupTransform(uav_name +"/world_origin", uav_name + "/utm_origin", tf2::TimePointZero);

          geometry_msgs::msg::PoseStamped pose_in, pose_out;
          pose_in.header.frame_id = frame_id;
          pose_in.pose.position = point.position;

          tf2::doTransform(pose_in, pose_out, transformStamped);

          // Replace with transformed coordinates
          point.position = pose_out.pose.position;

          frame_id = uav_name + "/world_origin";

          RCLCPP_INFO(get_logger(),
                       "Transformed waypoint from UTM to World: X: %.3f, Y: %.3f, Z: %.3f",
                       point.position.x, point.position.y, point.position.z);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN(get_logger(), "Transform failed in GetPathFromWaypoints: %s", ex.what());
        }
      }

      path.points.push_back(point);
    }

    path.header.frame_id = uav_name + "/world_origin";
    path.header.stamp = this->now();

    RCLCPP_INFO(get_logger(),
                "Generated path with %zu points in frame: %s",
                path.points.size(), path.header.frame_id.c_str());

    return path;
  }
  //}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(single_uav::SingleUAV);
