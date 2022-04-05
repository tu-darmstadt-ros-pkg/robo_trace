import psutil
import csv
import os
import time
import argparse


CSV_HEADER = ["process", "time", "cpu", "memory", "disk", "threads"]
CSV_BUFFER_SIZE = 128
RESOURCE_QUERY_RATE = 0.5

class ResourceUtilizationDumper():

    def __init__(self, names, file) -> None:

        # The approximate names of the processes to be monitored
        self.names = names
        # The names that could not be resolved to processes yet 
        self.pending = names.copy()

        self.processes = []

        print("Setting up resource utilization recorder ...")

        print(" + creating csv writer.")
        self.file_name = file
        os.makedirs(os.path.dirname(self.file_name), exist_ok=True)
        print(" + file is " + self.file_name)
       

        self.handle = open(
            file = self.file_name, 
            mode='w+',
            encoding="UTF8"
        )
        self.writer = csv.writer(self.handle)

        # The CSV header
        self.writer.writerow(CSV_HEADER)

        print("Setup complete.")

    def spin(self):
        
        print("Start spinning ...")

        while True:

            # Look for processes to be recorded
            self.scan_processes()
            # Dump stats
            self.dump_stats()

            time.sleep(RESOURCE_QUERY_RATE)
                   

    def scan_processes(self):
        
        if len(self.pending) == 0:
            return

        for process in psutil.process_iter():

            try:

                for name in self.pending:

                    if not name in process.name():
                        continue

                    self.processes.append(process)
                    self.pending.remove(name)

                    print(" + Found process " + process.name())

                    break
                
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                continue

            if len(self.pending) == 0:
                break

    def dump_stats(self):
        
        for process in self.processes:
            
            try:

                current_time = time.time()

                with process.oneshot():
                    # ["process", "time", "cpu", "memory", "disk", "threads"]
                    data = [
                        # name
                        process.name(),
                        # time
                        current_time,
                        # cpu
                        process.cpu_percent(),
                        # memory
                        process.memory_percent(),
                        # disk
                        0,
                        # threads
                        process.num_threads()
                    ]
                
                self.writer.writerow(data)

            except psutil.NoSuchProcess:
                # The process is no longer active.
                self.processes.remove(process)

def get_arguments():      

    parser = argparse.ArgumentParser(
        description="Records system utilization of processes to CSV files"
    )
    parser.add_argument("--file", 
        dest="file", 
        type=str, 
        default="utilization.csv",
        help="The file name to dump the utilization values to"
    )

    return parser.parse_args() 

if __name__ == '__main__':
    
    args = get_arguments()

    dumper = ResourceUtilizationDumper(
        names=["record", "mongo"],
        file=args.file
    )
    # Does all the work.
    dumper.spin()