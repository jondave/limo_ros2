source /opt/ros/lcas/install/setup.bash
source /workspaces/limo_ros2/install/setup.bash

unset RMW_IMPLEMENTATION
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/limo_ros2/.devcontainer/super_client_configuration_file.xml
export ROS_DISCOVERY_SERVER=localhost:11888
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE