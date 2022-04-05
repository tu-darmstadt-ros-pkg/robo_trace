from ast import arg
import os
import sys
import csv
import pymongo
import logging
import argparse
import json

import pandas as pd

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)

parser = argparse.ArgumentParser(
    description="Fetches message processing times from the database and dumps them to a CSV"
)
parser.add_argument("--file", 
    dest="file", 
    type=str, 
    default="processing_times.csv",
    help="The file name to dump to"
)
parser.add_argument("--database", 
    dest="db", 
    type=str, 
    default="robo_trace",
    help="The database to fetch from"
)
parser.add_argument("--metafile", 
    dest="metafile", 
    type=str, 
    default="db_meta_data.csv",
    help="The file name to write metedata to"
)
parser.add_argument('--drop', dest='drop', action='store_true')

args = parser.parse_args() 


logging.info("Connecting to MongoDB...")
client = pymongo.MongoClient(
    'localhost', 
    27017, 
    unicode_decode_error_handler='ignore'
)

logging.info("Connected!")
logging.info(f"Loading data for {args.db}")

db = client[args.db]
db_stats = db.command("dbstats")

metatdata = db_stats
metatdata["message_counts"] = {}

data_frames = []

for collection in db.list_collection_names():

    if ('evaluation' in collection) or (collection == "__metadata__") or ("blob" in collection):
        continue

    logging.info(f" - Loading {collection}")
    
    # Get all the data
    data_raw = list(db[collection].find())

    if len(data_raw) == 0:
        continue    
    
    metatdata["message_counts"][collection.replace("/", "_")] = len(data_raw) 
    data = pd.DataFrame(data_raw)
    # Get the timing
    data = data['metadata'].apply(pd.Series)["stamps"].apply(pd.Series)
    # Insert origin
    data["topic"] = collection

    data_frames.append(data)

if len(data_frames) == 0:
    logging.error("No data to be exported!")
else:

    data = pd.concat(data_frames)

    file_name = "processing_times.csv"
    logging.info("Writing data to file " + file_name)
        
    data.to_csv(
        args.file,
        index=False
    )

    with open(args.metafile, "w+") as dmf:
        dmf.write(json.dumps(metatdata))

    if args.drop:
        logging.info("Dropping database")
        client.drop_database(args.db)

    logging.info("Done!")

