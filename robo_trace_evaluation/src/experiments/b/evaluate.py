import os
import logging

import sys
sys.path.append('../../')

from loader import *
from parameters import *
from plotting import *

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)


def create_tag_provider(include_topic):

    def get_tags(name):

        tags = name.split("/")[-1].split("_")
        
        result = {
            'robot': tags[1],
            'idx': int(tags[2].replace(".csv", "")),
            'downsampling': int(tags[3].replace('D', '')),
            'encrypted': tags[4] == "E0"
        }

        if include_topic:
            result['topic'] = ('_'.join(tags[5:])).replace(".csv", "")
        
        return result

    return get_tags

def print_sys_data(sys):

    logging.info("System Utilization Stats")

    for topic in sys["topic"].unique():
        logging.info(f" + topic: {topic}")

        rdat = sys[sys["topic"] == topic]

        for encryption in sys["encrypted"].unique():
            logging.info(f"   = encryption: {encryption}")

            edata = rdat[rdat["encrypted"] == encryption]

            for downsampling in sys["downsampling"].unique():
                logging.info(f"     = downsampling: {downsampling}")

                ddat = edata[edata["downsampling"] == downsampling]

                cpu = ddat["cpu"].to_numpy()
                mem = ddat["memory"].to_numpy()

                logging.info(f"         CPU in %")
                logging.info(f"         CPU Mean: {np.mean(cpu)}")
                logging.info(f"         CPU Std: {np.std(cpu)}")
                logging.info(f"         CPU Med: {np.median(cpu)}")

                logging.info(f"         MEM in MB")
                logging.info(f"         MEM Mean: {np.mean(mem)}")
                logging.info(f"         MEM Std: {np.std(mem)}")
                logging.info(f"         MEM Med: {np.median(mem)}")

#
# Table on "System Utiliencryptedzation".
#

utilization_dump = get_dump(
    name="two",
    file_regex=f"{FILE_NAME_PREFIX_UTILIZATION}*.csv",
    processor=[
        processor_exclude_topics,
        processor_adjust_memory,
        processor_adjust_topic_name
    ],
    tag_extractor=create_tag_provider(True)
)

print_sys_data(utilization_dump)