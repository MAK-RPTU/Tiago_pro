#!/bin/bash

# Define the variable with the CycloneDDS XML configuration
local cfg_path="/opt/pal/alum/share/cyclone_dev_cfg/config/cyclone_config.xml"

# Just set the variables if CYCLONEDDS_URI is not set (no previous config)
if [ -z "$CYCLONEDDS_URI" ]; then
    # Use ament_prepend_unique_value to prepend the cyclone config value
    ament_prepend_unique_value CYCLONEDDS_URI "$cfg_path"

    # May interere with the config
    unset ROS_LOCALHOST_ONLY
    unset ROS_DOMAIN_ID

    # Set up the connection env flag
    ament_prepend_unique_value PAL_ROBOT_CONNECTED "0"

    # Set the ROS_DOMAIN_ID
    ament_prepend_unique_value ROS_DOMAIN_ID "1"
fi
