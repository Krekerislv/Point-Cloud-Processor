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

# TODO add arguments for eps and min_samples 
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

    # return arguments
    return args.input, args.algorithm, args.seperator, args.classifyNoise, args.cores, args.classification_column

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
    
def perform_DBSCAN(data):
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Starting DBSCAN...")

    # Create a DBSCAN object
    #db = DBSCAN(eps=0.49, min_samples=189, n_jobs=-1)
    db = DBSCAN(eps=0.5, min_samples=25, n_jobs=-1)

    # Perform DBSCAN
    db.fit(data)

    # change -1 (noise) to 255
    labels = db.labels_
    labels[labels == -1 ] = 255

    # add classification column to data
    data = np.column_stack([data,labels])

    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] DBSCAN finished!")

    print(f"\tNumber of clusters identified: {len(set(labels)) - (1 if 255 in labels else 0)}")
    print(f"\tTotal point count: {data.shape[0]}")
    print(f"\tNoise point count: {np.count_nonzero(labels == 255)}")

    return data

def perform_HDBSCAN_star(data):
    print("HDBSCAN")
    return data

# This should be done with txt2Potree, however... then i need to save to csv and pass it there. Would be better to import txt2Potree and use it
"""
def data_to_LAS(data):
    curTime = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    print(f"[{curTime}] Converting data to LAS!")

    # Init LAS file header
    header = laspy.LasHeader(point_format=6, version="1.4")
    header.offsets = np.min(data[: , :3], axis=0)
    header.scales = np.array([0.01, 0.01, 0.01])

    # Create a LAS object
    las = laspy.LasData(header)

    # Assign point coordinates
    las.x = data[:, 0]
    las.y = data[:, 1]
    las.z = data[:, 2]

    # Assign classification
    if data.shape[1] == 4:
        # If classification column was present in input file OR assigned with clustering algorithm
        las.classification = data[:, 3].astype(int)
    else:
        # If classificaton column is not present, set it to 0
        las.classification = np.zeros((data.shape[0]))

    # Write to LAS file
    LasFilePath = "tmp_las_file.las"
    
    las.write(LasFilePath)

    curTime = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    print(f"[{curTime}] LAS file created!")
    
    return LasFilePath

def LAS_to_Potree(las_path, Potree_converter_path, Potree_output, cloudName, classify_noise):
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Converting LAS to Potree...")

    # Define output_path
    if Potree_output == "DEFAULT":
        outputFolder = os.path.join(os.path.abspath(os.curdir), "Potree_files", cloudName)
    else:
        # Check if folder already exists (avoid overwrite)
        i = ""
        j = 1
        if classify_noise:
            c_str = "_cNoise"
        else:
            c_str = ""

        while os.path.exists(os.path.join(Potree_output, cloudName + c_str + i)):
            i = "_" + str(j)
            j += 1
        
        outputFolder = os.path.join(Potree_output, cloudName + c_str + i)


    #definē kommandrindas komandu, kuru izpildīt no Python
    command = [Potree_converter_path, las_path, "-o", outputFolder]

    #izpilda kommandu, neizvadot tās rezultātus Python terminālī
    subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Delete tmp LAS file
    os.remove(las_path)

    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Potree files generated at \"{outputFolder}\"")
"""

if __name__ == "__main__":
    # Main code
    
    # Get variables from config file
    POTREE_CONVERTER_PATH, POTREE_OUTPUT = parseConfig()

    # Validate command-line arguments
    INPUT_FILE, ALGORITHM, SEP, CLASSIFY_NOISE, CORES, CLASS_COLUMN = validateArguments()

    # Read text file
    data, fileName, ext = read_txt(INPUT_FILE, SEP)

    # if user specified -c or --classifyNoise then do so
    if CLASSIFY_NOISE:
        if ALGORITHM == "DBSCAN":
            data = perform_DBSCAN(data)
        elif ALGORITHM == "HDBSCAN*":
            data = perform_HDBSCAN_star(data)

    # Convert data to LAS
    las_path = txt2Potree.txtToLas(data, CLASS_COLUMN, fileName=os.path.splitext(os.path.basename(INPUT_FILE))[0])

    # Convert LAS to Potree
    txt2Potree.LasToPotree(las_path, POTREE_CONVERTER_PATH, POTREE_OUTPUT)