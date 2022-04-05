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
            'idx': int(tags[2].replace(".csv", ""))
        }

        if include_topic:
            result['topic'] = ('_'.join(tags[3:])).replace(".csv", "")
        
        return result

    return get_tags


#
# Tables on "Processing Share" and "Processing Frequency".
#

message_time_dump = get_dump(
    name="one",
    file_regex=f"{FILE_NAME_PREFIX_MESSAGE_TIMES}*.csv",
    processors=DEFAULT_PROCESSING_CHAIN_MESSAGE_TIMES,
    tag_extractor=create_tag_provider(False)
)

print_proc_share_simple(message_time_dump)
print_frequencies_simple(message_time_dump)

#
# Plot on "System Utilization" and "Processing Time"
#

utilization_dump = get_dump(
    name="one",
    file_regex=f"{FILE_NAME_PREFIX_UTILIZATION}*.csv",
    processors=DEFAULT_PROCESSING_CHAIN_UTILIZATION,
    tag_extractor=create_tag_provider(True)
)

processing_time_dump = get_dump(
    name="one",
    file_regex=f"{FILE_NAME_PREFIX_PROC_TIME}*.csv",
    processors=DEFAULT_PROCESSING_CHAIN_PROECESSING_TIMES,
    tag_extractor=create_tag_provider(False)
)

fig, axs = plt.subplots(
    nrows=2
)

plot_load(
    data=utilization_dump, 
    ax=axs[1],
    y_cpu_lim=30, 
    y_mem_scale=0.1
)
plot_proc_time(processing_time_dump, axs[0])

plt.tight_layout() 
plt.savefig(f"plt_utilization_and_proc_times.svg",format="svg")