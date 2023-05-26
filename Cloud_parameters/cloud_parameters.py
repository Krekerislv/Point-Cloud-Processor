# A program to display point cloud parametrs
import argparse
import os
from datetime import datetime
import json
import pandas as pd

def validateArguments():
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(prog="cloud_parametrs",description='Retrieve point cloud parameters from input.')

    # Add arguments to the parser
    parser.add_argument('-i', '--input', type=str, required=True, help='Input file')
    parser.add_argument('-s', '--seperator', type=str, required=False, default=',', help='Specify how data is seperated (default = \',\')')
    parser.add_argument('-c', '--classification_column', type=int, required=False, default=4, help='Specify which is the classification column (default = 4)')
    parser.add_argument('-p', '--print', action="store_false", required=False, help='Print data (default = True)')

    # Parse the command line arguments
    args = parser.parse_args()

    # Check if input file exists
    if not os.path.isfile(args.input):
        print(f"Input file \"{args.input}\" doesn't exist!")
        exit()
    
    # Check if seperator is valid
    if  len(args.seperator) != 1:
        print(f"Invalid seperator: \"{args.seperator}\"")
        exit()

    # Check if classification column is valid
    if args.classification_column <= 0:
        print(f"Invalid classification column specified: {args.classification_column}")
        exit()

    # return arguments
    return args.input, args.seperator, args.classification_column, args.print


def getCloudParams(input_file, sep, class_column, print_data=True):
    # Read input text file
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Loading data from file...")
    data = pd.read_csv(input_file, sep=sep, header=None).to_numpy()
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] File \"{os.path.basename(input_file)}\" loaded!")

    # Get total amount of points
    total_point_count = data.shape[0]

    # Get noise point count
    try:
        noise_point_count = data[data[: , class_column-1] == 255].shape[0]
    except:
        noise_point_count = 0

    # Get non-noise point count
    try:
        non_noise_count = data[data[: , class_column-1] != 255].shape[0]
    except:
        non_noise_count = 0

    # Get cluster count
    try:
        cluster_count = len(set(data[data[: , class_column-1] == 255][: , class_column-1]))
    except:
        cluster_count = 0

    # Setup dict
    info = {
        "total_point_count": total_point_count,
        "noise_point_count": noise_point_count,
        "non_noise_point_count": non_noise_count,
        "cluster_count": cluster_count
    }

    if print_data:
        print(json.dumps(info, indent=4))
    
    return info

if __name__ == "__main__":
    INPUT_FILE, SEP, CLASS_COLUMN, PRINT_DATA = validateArguments()
    info = getCloudParams(INPUT_FILE, SEP, CLASS_COLUMN, PRINT_DATA)