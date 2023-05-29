# A program to display point cloud parametrs
import argparse
import os
from datetime import datetime
import json
import pandas as pd
import numpy as np

def validateArguments():
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(prog="cloud_parametrs",description='Retrieve point cloud parameters from input.')

    # Add arguments to the parser
    parser.add_argument('-i', '--input', type=str, required=True, help='Input file')
    parser.add_argument('-s', '--seperator', type=str, required=False, default=',', help='Specify how data is seperated (default = \',\')')
    parser.add_argument('-c', '--classification_column', type=int, required=False, default=4, help='Specify which is the classification column (default = 4)')
    parser.add_argument('-p', '--print', action="store_false", required=False, help='Print data (default = True)')
    parser.add_argument('-gt', '--ground_truth', type=str, required=False, default=None, help='Specify ground truth data to calculate noise classification accuracy')

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
    return args.input, args.seperator, args.classification_column, args.print, args.ground_truth


def getCloudParams(input_file, sep, class_column, print_data=True, ground_truth_path=None):
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
        cluster_count = len(set(data[data[: , class_column-1] != 255][: , class_column-1]))
    except:
        cluster_count = 0

    # Setup dict
    info = {
        "total_point_count": total_point_count,
        "noise_point_count": noise_point_count,
        "non_noise_point_count": non_noise_count,
        "cluster_count": cluster_count
    }

    # Get noise classification accuracy score
    if ground_truth_path != None:
        curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
        print(f"[{curTime}] Loading ground_truth from file...")
        ground_truth = pd.read_csv(ground_truth_path, sep=sep, header=None).to_numpy()

        curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
        print(f"[{curTime}] Ground truth file \"{os.path.basename(ground_truth_path)}\" loaded!")

        ground_truth_dict = {tuple(row[:class_column-1]): row[class_column-1] for row in ground_truth}

        # Extract the correct labels from the dictionary based on the data values
        true_labels = np.array([ground_truth_dict.get(tuple(row[:class_column-1])) for row in data])
        
        if true_labels[true_labels == None].shape[0] != 0:
            print(f"Warning! Ground truth doesn't contain all points of input dataset!\nNoise classification precision score might be inaccurate.")

        labels = data[: , class_column-1]

        correct_noise_point_count = np.count_nonzero((true_labels == 255) & (labels == 255))
        incorrect_noise_point_count = np.count_nonzero((true_labels != 255) & (labels == 255))
    
        acc = (correct_noise_point_count - incorrect_noise_point_count) / noise_point_count

        info["noise_classification_accuracy_score"] = acc
    

    if print_data:
        print(json.dumps(info, indent=4))
    
    return info

if __name__ == "__main__":
    INPUT_FILE, SEP, CLASS_COLUMN, PRINT_DATA, GROUND_TRUTH = validateArguments()
    info = getCloudParams(INPUT_FILE, SEP, CLASS_COLUMN, PRINT_DATA, GROUND_TRUTH)