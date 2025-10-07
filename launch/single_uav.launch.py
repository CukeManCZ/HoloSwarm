import launch
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription()
    
    pkg_name = "holo_swarm"
    this_pkg_path = get_package_share_directory(pkg_name) 
    namespace = "holo_swarm"

    uav_name = os.getenv("UAV_NAME", "uav1")
    use_sim_time=os.getenv("USE_SIM_TIME", "false") == "true"
   

    single_uav_node = ComposableNode(
                    package=pkg_name,
                    plugin="single_uav::SingleUAV",
                    namespace=uav_name,
                    name="singleUAV",
                    parameters=[
                        {"uav_name": uav_name},
                        {"topic_prefix":"/"+uav_name},
                        {"enable_profiler": False},
                        {"use_sim_time":use_sim_time},
                        #{"config": this_pkg_path + "/config/waypoint_follower.yaml"}
                    ],
                    remappings=[
                        ("/get_odom", "estimation_manager/odom_main"),
                        ("/get_poses", "control_manager/trajectory_original/poses"),
                        ("/get_boundaries", "control_manager/safety_area_coordinates_markers"),
                        ("/set_trajectory", "trajectory_generation/path"),
                        ("/start_trajectory_tracking", "control_manager/start_trajectory_tracking"),
                        ("/stop_trajectory_tracking", "control_manager/stop_trajectory_tracking"),
                        ("/set_reference", "control_manager/reference"),

                        ("/set_takeoff", "uav_manager/takeoff"),
                        ("/set_land", "uav_manager/land"),
                        ("/set_home", "uav_manager/land_home"),
                    ]
    )
  
    standalone_container = ComposableNodeContainer(
            namespace=uav_name, 
            name=namespace+"_container",
            package="rclcpp_components",
            executable="component_container_mt", 
            output="screen",
            composable_node_descriptions=[single_uav_node],
    )

    ld.add_action(standalone_container)

    return ld
