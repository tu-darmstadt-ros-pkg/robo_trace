import os
import logging

import sys
sys.path.append('../../')

from runner import *
from parameters import *

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)


TOPICS_TO_ANALYZE = [
    "/autonomy_module_lidar/lidar_packets",
    "/camera360/image_raw/compressed",
    "/scan_cloud_filtered"
]

def get_id(experiment, repetition):
    return f"{experiment['robot']}_{repetition}_D{int(float(experiment['downsampling']))}_E{experiment['encrypted']}_{experiment['topics'][0].split('/')[-1]}"


experiments = []

for descriptor in BAG_FILE_DESCRIPTIONS:
    for topic in descriptor["topics"]:

        if not topic in TOPICS_TO_ANALYZE:
            continue

        # Sweeps
        for encryption in ["0", "7"]:
            for downsampling in ["1.0", "4.0"]:

                experiments.append({
                    'launch': {
                        'file': 'params.launch',
                        'args': [
                            f"downsampling_value:={downsampling}",
                            # We simply enable/disable encryption by adjusting the machting priority for the pipelines. 
                            f"general_pipeline_priority:={encryption}"
                        ]
                    },
                    'topics': [topic],
                    'file': descriptor['file'],
                    'duration': descriptor['duration'],
                    'arrivals': True, 
                    'utilization': True,
                    'downsampling': downsampling,
                    'encrypted': encryption,
                    'robot': descriptor['robot']
                })

run_experiment(
    name="two",
    experiements=experiments,
    repetitions=5,
    get_id=get_id
)