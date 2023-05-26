import configparser
import argparse
import os
import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
from datetime import datetime
import sys

# Add temp path variable and import txt2Potree
current_dir = os.path.dirname(os.path.abspath(__file__))
txt2Potree_dir = os.path.join(current_dir, '..', 'txt2Potree')
sys.path.append(txt2Potree_dir)
import txt2Potree
 
def validateArguments():
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(description='Convert text files to Potree format.')

    # Add arguments to the parser
    parser.add_argument('-i', '--input', type=str, required=True, help='Input file')
    parser.add_argument('-s', '--seperator', type=str, required=False, default=',', help='Specify how data is seperated (default = \',\')')
    parser.add_argument('-c', '--classifyNoise', action='store_true', help="Add this flag to classify noise points.")
    parser.add_argument('-a', '--algorithm', type=str, choices=['DBSCAN', 'HDBSCAN*'], required=False, help='Clustering algorithm to classify noise points with.')
    parser.add_argument('--cores', type=int, required=False, default=4, help='Number of CPU cores to use in parallel (default = 4). Use -1 for all available')
    parser.add_argument('--classification_column', type=int, required=False, default=4, help='Specify which is the classification column (default = 4)')

    # Arguments for DBSCAN
    parser.add_argument('--eps', type=float, required=False, help="Specify the value of eps for DBSCAN (only if using DBSCAN).")
    parser.add_argument('--min_samples', type=int, required=False, help="Specify the value of min samples for DBSCAN (only if using DBSCAN).")

    # Parse the command line arguments
    args = parser.parse_args()

    # If classifyNoise flag is true
    if args.classifyNoise and args.algorithm is None:
        parser.error("If '-c' or '--classifyNoise' is present, algorithm must be specified using '-a' or '--algorithm'.")

    # Check if input file exists
    if not os.path.isfile(args.input):
        print(f"Input file \"{args.input}\" doesn't exist!")
        exit()

    # Check if seperator is valid
    if len(args.seperator) != 1:
        print(f"Invalid seperator: \"{args.seperator}\"")
        exit()

    # Check for valid core count
    if args.cores <= 0 and args.cores != -1:
        print(f"Argument \"--cores\" should be positive integer or -1! You specified: {args.cores}")

    # check DBSCAN arguments
    if args.algorithm == "DBSCAN":
        if not args.eps or not args.min_samples:
            print("If using DBSCAN, --eps and --min_samples must be specified!")
            exit()
        
        if args.eps <= 0:
            print(f"Specified invalid eps value of {args.eps}. Eps must be > 0")
            exit()
        if args.min_samples <= 0:
            print(f"Specified invalid min samples value of {args.min_samples}. Min samples must be > 0")
            exit()

    # return arguments
    return args.input, args.algorithm, args.seperator, args.classifyNoise, args.cores, args.classification_column, args.eps, args.min_samples

def parseConfig():
    config = configparser.ConfigParser()
    config.read("./config.ini")

    # Read PotreeConverter path from config.ini
    PC_path = config["PotreeConverter"]["path"]

    # Check if PotreeConverter exists
    if not os.path.isfile(PC_path):
        print(f"PotreeConverter file \"{PC_path}\" doesn't exist!")
        exit()
    # Output path
    Potree_output_path = config["PotreeConverter"]["output"]

    # If output directory doesn't exist, create it
    if not os.path.isdir(Potree_output_path):
        os.makedirs(Potree_output_path)

    return PC_path, Potree_output_path

def read_txt(file, sep):
    # Read data from text file
    data = pd.read_csv(file, sep=sep, header=None)

    # Remve duplicates
    data = data.drop_duplicates()
    
    # Convert data to numpy
    data = data.to_numpy()

    # Get file name
    fileName, ext = os.path.splitext(os.path.basename(file))
    
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] File \"{os.path.basename(file)}\" read successfuly!")
    
    return data, fileName, ext
    
def perform_DBSCAN(data, eps, min_samples, n_jobs, cc):
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Starting DBSCAN...")

    # Create a DBSCAN object
    db = DBSCAN(eps=eps, min_samples=min_samples, n_jobs=n_jobs)

    # Perform DBSCAN
    db.fit(np.delete(data,[cc-1], axis=1))

    # change -1 (noise) to 255
    labels = db.labels_
    labels[labels == -1 ] = 255

    # add classification column to data
    data[: , cc-1] = labels

    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] DBSCAN finished!")

    print(f"\tNumber of clusters identified: {len(set(labels)) - (1 if 255 in labels else 0)}")
    print(f"\tTotal point count: {data.shape[0]}")
    print(f"\tNoise point count: {np.count_nonzero(labels == 255)}")

    return data

def perform_HDBSCAN_star(data):
    print("TODO: HDBSCAN*")

if __name__ == "__main__":
    # Main code
    
    # Get variables from config file
    POTREE_CONVERTER_PATH, POTREE_OUTPUT = parseConfig()

    # Validate command-line arguments
    INPUT_FILE, ALGORITHM, SEP, CLASSIFY_NOISE, CORES, CLASS_COLUMN, EPS, MIN_SAMPLES = validateArguments()

    # Read text file
    data, fileName, ext = read_txt(INPUT_FILE, SEP)

    # if user specified -c or --classifyNoise then do so
    if CLASSIFY_NOISE:
        if ALGORITHM == "DBSCAN":
            data = perform_DBSCAN(data, EPS, MIN_SAMPLES, CORES, cc=CLASS_COLUMN)
        elif ALGORITHM == "HDBSCAN*":
            data = perform_HDBSCAN_star(data)
        
        fileName += "_cleaned"

    # Convert data to LAS
    las_path = txt2Potree.txtToLas(data, CLASS_COLUMN, fileName=fileName)

    # Convert LAS to Potree
    txt2Potree.LasToPotree(las_path, POTREE_CONVERTER_PATH, POTREE_OUTPUT)