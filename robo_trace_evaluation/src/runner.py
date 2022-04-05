from cmath import log
import subprocess
import signal
import os
import logging
import time

from parameters import *


PACKAGE_BASE_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def run_experiment(name, experiements, repetitions, get_id):
    
    #
    logging.info(f"Starting {len(experiements)} experiments with {repetitions} cycles.")
    #
    experiment_result_directory = os.path.join(PACKAGE_BASE_PATH, PATH_DUMPS_DIRECTORY, name)

    #
    for repetition in range(repetitions):
        #
        logging.info(f" + Entering repetition {repetition}/{repetitions}.")
        #
        for experiment in experiements:
            
            id = get_id(experiment, repetition)
            logging.info(f"  + Starting experiment {id}.")
            logging.info(f"    + Spinning up environment.")

            environment_args = [
                "roslaunch", "robo_trace",  experiment["launch"]["file"]
            ]

            if "args" in experiment["launch"]:
                for arg in experiment["launch"]["args"]:
                    environment_args.append(arg)

            process_ros_environment = subprocess.Popen(
                args=environment_args,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT  
            )

            if RUNNER_PROCESS_STARTUP_WAIT_TIME is not None and RUNNER_PROCESS_STARTUP_WAIT_TIME > 0:
                time.sleep(RUNNER_PROCESS_STARTUP_WAIT_TIME)

            recorder_args =  [
                "rosrun", "robo_trace", "record", 
                    "--db_timeout", str(5000), 
                    "--db_name", f"recording_{id}"
                ]

            if "topics" in experiment:
                # 
                recorder_args.append("--topic")
                #
                for topic in experiment["topics"]:
                    recorder_args.append(topic)
            else:
                recorder_args.append("-a")

            logging.info(f"    + Spinning up recorder.")

            process_recorder = subprocess.Popen(
                args=recorder_args,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT     
            )

            if RUNNER_PROCESS_STARTUP_WAIT_TIME is not None and RUNNER_PROCESS_STARTUP_WAIT_TIME > 0:
                time.sleep(RUNNER_PROCESS_STARTUP_WAIT_TIME)

            # Dump message arrival times if wanted
            if "arrivals" in experiment:
                
                logging.info(f"    + Spinning up arrival time dumper.")

                process_dumper_arrivals = subprocess.Popen(
                    args=[
                        "rosrun", "robo_trace_evaluation", "dump_arrival_times.py",
                            "--file", os.path.join(PATH_DUMPS_DIRECTORY,  os.path.join(experiment_result_directory, f"{FILE_NAME_PREFIX_MESSAGE_TIMES}_{id}.csv"))
                    ],
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT  
                )

                if RUNNER_PROCESS_STARTUP_WAIT_TIME is not None and RUNNER_PROCESS_STARTUP_WAIT_TIME > 0:
                    time.sleep(RUNNER_PROCESS_STARTUP_WAIT_TIME)

            rosbag_args = [
                    "rosbag", "play", os.path.join(PACKAGE_BASE_PATH, PATH_BAGS_DIRECTORY, experiment["file"]),
                        "--queue=100000"
                ]
            
            if "topics" in experiment:
                # 
                rosbag_args.append("--topics")
                #
                for topic in experiment["topics"]:
                    rosbag_args.append(topic)

            logging.info(f"    + Spinning up rosbag.")
            
            process_bag = subprocess.Popen(
                args=rosbag_args,
                preexec_fn=os.setsid,
                # RosBag won't playback if it can't dump its stdout.
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT  
            )

            # 
            execution_start_time = time.time()

            if "utilization" in experiment:
                
                logging.info(f"    + Spinning up utilization dumper.")

                process_dumper_utilization = subprocess.Popen(
                    args=[
                        "python3", os.path.join(PACKAGE_BASE_PATH, "src", "utilization.py"),
                            "--file", os.path.join(experiment_result_directory, f"{FILE_NAME_PREFIX_UTILIZATION}_{id}.csv")
                    ],
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT  
                )
            
            while (process_bag.returncode is None) and (
                # Enforce a upper limit time bound if wanted.
                (not "duration" in experiment) or (time.time() - execution_start_time < experiment["duration"])
            ):  
                logging.info(f"    + Waiting for completion (Elapsed: {time.time() - execution_start_time} / Code: {process_bag.returncode}).")
                time.sleep(RUNNER_TERMINATION_CHECK_POLLING_TIME)

            logging.info(f"    + Completed. Terminating processes.")

            # 
            os.killpg(os.getpgid(process_recorder.pid), signal.SIGTERM)
            
            if RUNNER_PROCESS_TERMINATION_WAIT_TIME is not None and RUNNER_PROCESS_TERMINATION_WAIT_TIME > 0:
                time.sleep(RUNNER_PROCESS_TERMINATION_WAIT_TIME)

            if "utilization" in experiment:
                os.killpg(os.getpgid(process_dumper_utilization.pid), signal.SIGTERM)

            if RUNNER_PROCESS_TERMINATION_WAIT_TIME is not None and RUNNER_PROCESS_TERMINATION_WAIT_TIME > 0:
                time.sleep(RUNNER_PROCESS_TERMINATION_WAIT_TIME)

            if "arrivals" in experiment:
                os.killpg(os.getpgid(process_dumper_arrivals.pid), signal.SIGTERM)

            if RUNNER_PROCESS_TERMINATION_WAIT_TIME is not None and RUNNER_PROCESS_TERMINATION_WAIT_TIME > 0:
                            time.sleep(RUNNER_PROCESS_TERMINATION_WAIT_TIME)

            os.killpg(os.getpgid(process_bag.pid), signal.SIGTERM)

            if RUNNER_PROCESS_TERMINATION_WAIT_TIME is not None and RUNNER_PROCESS_TERMINATION_WAIT_TIME > 0:
                            time.sleep(RUNNER_PROCESS_TERMINATION_WAIT_TIME)

            os.killpg(os.getpgid(process_ros_environment.pid), signal.SIGTERM)  

            if RUNNER_PROCESS_TERMINATION_WAIT_TIME is not None and RUNNER_PROCESS_TERMINATION_WAIT_TIME > 0:
                time.sleep(RUNNER_PROCESS_TERMINATION_WAIT_TIME)

            logging.info(f"    + Converting message processing times to CSV.")
            
            # TODO: Kind of ugly. We need to run this script though for in order to delete the associated database
            #       and keep the (my) system from overflowing. :) 
            subprocess.run([
                "python3", os.path.join(PACKAGE_BASE_PATH, "src", "export.py"),
                    "--file", os.path.join(experiment_result_directory, f"{FILE_NAME_PREFIX_PROC_TIME}_{id}.csv"),
                    "--metafile", os.path.join(experiment_result_directory, f"{FILE_NAME_PREFIX_METADATA}_{id}.csv"),
                    "--database", f"recording_{id}",
                    "--drop"
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT  
            )