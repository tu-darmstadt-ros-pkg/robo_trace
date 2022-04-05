# RoboTrace Evaluation
This repository contains the source for evaluating RoboTrace.

## Dependencies And Setup
As stated in the main repository, RoboTrace was developed for ROS `melodic`. Consequently, it is also mandatory  running this evaluation code. Besides this, the following additional dependecies must be also installed:

- The python3 packages as listed in the `requirements.txt`.

After all dependecies have been installed successfully, build the project and source the workspace. The evaluation code should be now ready for usage.

## Usage
In the `/src/experiments` directory 3 subfolders can be found, each containing the sources for one experiment from the paper. Here, the `execute.py` script executes the experiment itself i.e. collectes the data to CSV files, while the `evaluate.py` script generates metrics or graphs from this data. Note that for running these experiments, the necessary bag files must be downloaded separately and placed in the `/bags` directoy. 