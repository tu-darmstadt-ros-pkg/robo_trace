import os
import logging

import sys
sys.path.append('../../')

from runner import *
from parameters import *

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)


def get_id(experiment, repetition):
    # There is only one topic
    return f"{experiment['robot']}_{repetition}_{experiment['topics'][0].split('/')[-1]}"


experiments = []

for descriptor in BAG_FILE_DESCRIPTIONS:
    for topic in descriptor["topics"]:
        experiments.append({
            'launch': {
                'file': 'params.launch'
            },
            'topics': [topic],
            'file': descriptor['file'],
            'duration': descriptor['duration'],
            'arrivals': True, 
            'utilization': True,
            'robot': descriptor['robot']
        })

run_experiment(
    name="one",
    experiements=experiments,
    repetitions=5,
    get_id=get_id
)