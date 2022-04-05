import os
import logging

import sys
sys.path.append('../../')

from loader import *
from parameters import *
from plotting import *

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)


def get_tags(name):

    tags = name.split("/")[-1].split("_")
    
    result = {
        'robot': tags[1],
        'idx': int(tags[2].replace(".csv", ""))
    }

    return result


#
# Tables on "Processing Share" and "Processing Frequency".
#

message_time_dump = get_dump(
    name="three",
    file_regex=f"{FILE_NAME_PREFIX_MESSAGE_TIMES}*.csv",
    processors=DEFAULT_PROCESSING_CHAIN_MESSAGE_TIMES,
    tag_extractor=get_tags
)

print_proc_share_simple(message_time_dump)
print_frequencies_simple(message_time_dump)

#
# Table on "System Utilization".
#

utilization_dump = get_dump(
    name="three",
    file_regex=f"{FILE_NAME_PREFIX_UTILIZATION}*.csv",
    processors= [
        processor_adjust_memory
    ],
    tag_extractor=get_tags
)

def print_system_utilization(data):

    logging.info("System Utilization Stats")
    for robot in data["robot"].unique():
        logging.info(f" + {robot}")

        rdat = data[data["robot"] == robot]

        cpu = rdat["cpu"].to_numpy()
        logging.info(f"    + CPU Load [%]")
        logging.info(f"       - Mean: {np.mean(cpu)}")
        logging.info(f"       - Std: {np.std(cpu)}")
        logging.info(f"       - Med: {np.median(cpu)}")

        memory = rdat["memory"].to_numpy()
        logging.info(f"    + Memory Utilization [MB]")
        logging.info(f"       - Mean: {np.mean(memory)}")
        logging.info(f"       - Std: {np.std(memory)}")
        logging.info(f"       - Med: {np.median(memory)}")

print_system_utilization(utilization_dump)

#
# Table on "Processing Time".
#

processing_time_dump = get_dump(
    name="three",
    file_regex=f"{FILE_NAME_PREFIX_PROC_TIME}*.csv",
    processors=DEFAULT_PROCESSING_CHAIN_PROECESSING_TIMES,
    tag_extractor=get_tags
)

def print_system_utilization(data):

    logging.info("Processing Time Stats [µs]")
    for topic in data["topic"].unique():

        if not topic in NAME_MAPPING:
            continue

        logging.info(f" + {NAME_MAPPING[topic]}")

        tdata = data[data["topic"] == topic]
        pdata = tdata["processing"]

        logging.info(f"    - Mean: {pdata.mean()}")

print_system_utilization(processing_time_dump)

#
# General System Metadata.
#

def get_metadata(name, file_regex, tag_extractor):

    experiment_result_directory = os.path.join(PACKAGE_BASE_PATH, PATH_DUMPS_DIRECTORY, name, file_regex)
    metadata = {}

    for file in glob.glob(experiment_result_directory):
      
        try:
            with open(file) as handle:
                data = json.load(handle)
        except:
            continue
        
        tags = tag_extractor(file)
        robot = tags["robot"]

        if not robot in metadata:
            metadata[robot] = {
                'sizes': [data['totalSize']],
                'objs': [data['objects']]
            }
        else:
            metadata[robot]['sizes'].append(data['totalSize'])
            metadata[robot]['objs'].append(data['objects'])

    return metadata

def print_system_metadata(data):
    logging.info("Database Size Stats [MB]")

    for robot in data:
        
        sizes = np.array(data[robot]['sizes'])
        
        logging.info(f" + {robot}")
        logging.info(f"   - Mean: {np.mean(sizes)}")
        logging.info(f"   - Std: {np.std(sizes)}")
        logging.info(f"   - Med: {np.median(sizes)}")

metadata_dump = get_metadata(
    name="three", 
    # TODO: Ups. Should be a json file.
    file_regex=f"{FILE_NAME_PREFIX_METADATA}*.csv", 
    tag_extractor=get_tags
)