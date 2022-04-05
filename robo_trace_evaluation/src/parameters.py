#!/usr/bin/env python3

import os
import rospkg
import psutil

import seaborn as sns

# The memory that is availabe on the evaluation system in MB. 
SYSTEM_TOTAL_MEMORY = psutil.virtual_memory().total / 1000000

# The base path of the evaluation
PATH_DUMPS_DIRECTORY = "dumps"
#
PATH_BAGS_DIRECTORY = "bags"

#
FILE_NAME_PREFIX_MESSAGE_TIMES = "arrivals"
#
FILE_NAME_PREFIX_PROC_TIME = "processing"
#
FILE_NAME_PREFIX_UTILIZATION = "utilization"
#
FILE_NAME_PREFIX_METADATA = "metadata"

# 
RUNNER_PROCESS_STARTUP_WAIT_TIME = 2
#
RUNNER_TERMINATION_CHECK_POLLING_TIME = 5
#
RUNNER_PROCESS_TERMINATION_WAIT_TIME = 2

# 
LOADER_RECORDER_SIGNALING_PREFIX = "/robo_trace/signal_stored"


#
TOPICS_EXCLUDED = [
    "xsens", 
    "data",
    "camera_info", 
    "tf_static", 
    "/rosout", 
    "/rosout_agg", 
    "/clock", 
    "/robo_trace/signal_stored/clock"
]

# 
PALETTE_TUD = sns.color_palette([
    "#5D85C3",
    "#009CDA",
    "#50B695",
    "#AFCC50",
    "#DDDF48",
    "#FFE05C",
    "#F8BA3C",
    "#EE7A34",
    "#E9593E",
    "#C9308E",
    "#804597"
])

# Mapps "shortened_topic_name@robot_name" to the label used in the paper
NAME_MAPPING = {
    'compressed@scout': 'image_360',
    'image_raw@tiago': 'image',
    'imu@scout': 'imu',
    'joint_states@scout': 'joint_states', 
    'lidar_packets@scout': 'lidar_360', 
    'odom@scout': 'odom', 
    'scan_cloud_filtered@scout': 'point_cloud',
    'scan_raw@tiago': 'lidar', 
    'sonar_base@tiago': 'sonar',
    'tf@scout': 'tf_scout',
    'tf@tiago': 'tf_tiago',
}

#
BAG_FILE_DESCRIPTIONS = [
    {
        'file': "2021-06-25-11-53-05.orig.bag",
        'duration': 80,
        'robot': "scout",
        'topics': [ 
            "/autonomy_module_lidar/lidar_packets",
            "/scan_cloud_filtered",
            "/camera360/camera_info",
            "/camera360/image_raw/compressed",
            "/imu/xsens/data",
            "/joint_states",
            "/odom",
            "/os_cloud_nodelet/imu",
            "/tf"
        ]
    },
    {
        'file': "2021-07-20-11-43-00.bag",
        'duration': 75,
        'robot': "tiago",
        'topics': [ 
            "/scan_raw",
            "/sonar_base",
            "/tf",
            "/xtion/rgb/image_raw"
        ]
    }
]