import json
import os
import sys
import csv
import logging
import glob

import pandas as pd

from parameters import *


PACKAGE_BASE_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def processor_exclude_topics(frame):
    
    for exclusion in TOPICS_EXCLUDED:
        frame = frame[frame["topic"].str.contains(exclusion) == False] 

    return frame

def processor_adjust_topic_name(frame):
    frame["topic"] = frame["topic"].str.split("/").str[-1] + "@" + frame["robot"]
    return frame

def processor_adjust_memory(frame): 
    # We want to convert to MB (*16000) and the column is given in 
    # he interval [0, 100], so we need to scale it down to [0, 1].   
    frame["memory"] = frame["memory"] * SYSTEM_TOTAL_MEMORY / 100
    return frame

def processor_calculate_processing_time(frame):
    frame["processing"] = frame["egress"] - frame["ingress"]
    return frame
  

def processor_extract_message_source(frame):

    frame_actual = frame["topic"].str.contains(LOADER_RECORDER_SIGNALING_PREFIX) == False
    
    if len(frame_actual) > 0:
        frame.loc[frame_actual, "source"] = "actual"

    fram_processed = frame["topic"].str.contains(LOADER_RECORDER_SIGNALING_PREFIX) == True
    
    if len(fram_processed) > 0:
        frame.loc[fram_processed, "topic"] = frame[fram_processed]["topic"].str.replace(LOADER_RECORDER_SIGNALING_PREFIX, "")
        frame.loc[fram_processed, "source"] = "processed" 

    return frame

def processor_calculate_processing_frequency(frame):

    for topic in frame["topic"].unique():
        selector = frame["topic"] == topic
        frame.loc[selector, "period"] = frame.loc[selector]["time"].diff(periods=1).rolling(10).mean()
    
    frame["hz"] = 1 / frame["period"]

    return frame

DEFAULT_PROCESSING_CHAIN_UTILIZATION = [
    processor_exclude_topics, 
    processor_adjust_memory, 
    processor_adjust_topic_name
]

DEFAULT_PROCESSING_CHAIN_MESSAGE_TIMES = [
    processor_exclude_topics, 
    processor_calculate_processing_frequency, 
    processor_extract_message_source,
    processor_adjust_topic_name
]

DEFAULT_PROCESSING_CHAIN_PROECESSING_TIMES = [
    processor_exclude_topics, 
    processor_calculate_processing_time, 
    processor_adjust_topic_name
]

def get_dump(name, file_regex, processors, tag_extractor):

    logging.info(f"Loading data for experiment {name} at {file_regex}")
    experiment_result_directory = os.path.join(PACKAGE_BASE_PATH, PATH_DUMPS_DIRECTORY, name, file_regex)
    frames = []

    for file in glob.glob(experiment_result_directory):
        
        logging.info(f" + {file}")

        try:
            frame = pd.read_csv(file)
        except:
            continue
        
        if tag_extractor is not None:

            tags = tag_extractor(file)

            if tags is not None:
                for tag in tags:
                    frame[tag] = tags[tag]

        if processors is not None:
            for processor in processors:

                frame = processor(frame)

                if frame is None or len(frame) == 0:
                    break
        
        if frame is None or len(frame) == 0:
            continue
    
        frame_cleaned = frame.dropna()

        if len(frame_cleaned) == 0:
            continue

        frames.append(frame_cleaned)

    if len(frames) == 0:
        return 

    return pd.concat(frames)