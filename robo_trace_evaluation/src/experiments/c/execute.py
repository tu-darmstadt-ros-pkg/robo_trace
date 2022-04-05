import os
import rospkg
import logging

import sys
sys.path.append('../../')

from runner import *
from parameters import *

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)


def get_id(experiment, repetition):
    # There is only one topic
    return f"{experiment['robot']}_{repetition}"

experiments = []

for descriptor in BAG_FILE_DESCRIPTIONS:
    experiments.append({
        'launch': {
            'file': 'params.launch'
        },
        'file': descriptor['file'],
        'duration': descriptor['duration'],
        'arrivals': True, 
        'utilization': True,
        'robot': descriptor['robot']
    })

run_experiment(
    name="three",
    experiements=experiments,
    repetitions=5,
    get_id=get_id
)