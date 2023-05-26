import laspy
import numpy as np
import pandas as pd
import os
import subprocess
import configparser
import argparse
from datetime import datetime

def validateArguments():
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(prog="txt2Potree",description='Convert point clouds from text file to Potree.')

    # Add arguments to the parser
    parser.add_argument('-i', '--input', type=str, required=True, help='Input file', dest="input")
    parser.add_argument('-s', '--seperator', type=str, required=False, default=',', help='Specify how data is seperated (default = \',\')', dest="seperator")
    parser.add_argument('-c', '--classification_column', type=int, required=False, default=4, help='Specify which is the classification column (default = 4)')

    # Parse the command line arguments
    args = parser.parse_args()

    # Check if input file exists
    if not os.path.isfile(args.input):
        print(f"Input file \"{args.input}\" doesn't exist!")
        exit()

    if  len(args.seperator) != 1:
        print(f"Invalid seperator: \"{args.seperator}\"")
        exit()

    # return arguments
    return args.input, args.seperator, args.classification_column

def parseConfig():
    config = configparser.ConfigParser()
    config.read("./config.ini")

    # Read PotreeConverter path from config.ini
    PC_path = config["PotreeConverter"]["path"]

    # Check if Potree Converter exists
    if not os.path.isfile(PC_path):
        print(f"Invalid PotreeConverter path specified in config.ini: \"{PC_path}\"")
        exit()
    
    # Output path
    Potree_output_path = config["PotreeConverter"]["output"]

    # Check for default case
    if Potree_output_path != "DEFAULT":
        try:
            Potree_output_path = os.path.normpath(Potree_output_path)
        except Exception as e:
            raise e

    return PC_path, Potree_output_path

def txtToLas(data, cc, fileName="tmp_las.las"):
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Converting data to .las file...")

    # Check for extension
    if fileName[-4:len(fileName)] != ".las":
        fileName += ".las"

    # Init LAS file header
    header = laspy.LasHeader(point_format=6, version="1.4")
    header.offsets = np.min(data[: , :3], axis=0)
    header.scales = np.array([0.01, 0.01, 0.01])

    # Create a LAS file
    las = laspy.LasData(header)

    # Assign point coordinates
    las.x = data[:, 0]
    las.y = data[:, 1]
    las.z = data[:, 2]

    # Assign classification
    try:
        las.classification = data[:, cc-1].astype(int)
    except:
        pass

    # Write to LAS file
    LasFile = os.path.join(os.path.abspath(os.curdir), fileName)
    las.write(LasFile)

    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Successfully generated .las file!\n")

    return LasFile

def LasToPotree(lasFile, potreeConverterPath, outputPath):
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Converting .las file to Potree...")

    # Define output_path
    if outputPath == "DEFAULT":
        outputFolder = os.path.abspath(os.curdir) + "/Potree_files/" + os.path.splitext(os.path.basename(lasFile))[0]
    else:
        outputFolder = os.path.join(outputPath, os.path.splitext(os.path.basename(lasFile))[0])

    # Define terminal command
    command = [potreeConverterPath, lasFile, "-o", outputFolder]

    # Execute command without printing output
    subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Remove tmp las file
    os.remove(lasFile)

    # Validate results
    expectedFiles = ["hierarchy.bin", "metadata.json", "octree.bin"]
    for file in expectedFiles:
        if not os.path.isfile(os.path.join(outputFolder, file)):
            raise Exception("PotreeConverter failed!")
    
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Potree files generated at {outputFolder}\n")

    return outputFolder


if __name__ == "__main__":
    # Parse arguments
    INPUT_FILE, SEPERATOR, CLASSIFICATION_COLUMN = validateArguments()

    # Parse config.ini
    POTREE_CONVERTER, OUTPUT_DIR = parseConfig()

    # Get file name and extension
    fileName, ext = os.path.splitext(os.path.basename(INPUT_FILE))

    # Read input file
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] Loading data from file...")

    points = pd.read_csv(INPUT_FILE, sep=SEPERATOR, header=None).to_numpy()
    
    curTime = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
    print(f"[{curTime}] File \"{os.path.basename(INPUT_FILE)}\" loaded!")

    # Convert to .las
    lasFile = txtToLas(points, CLASSIFICATION_COLUMN, fileName=fileName)

    # Convert to Potree
    PotreePath = LasToPotree(lasFile=lasFile, potreeConverterPath=POTREE_CONVERTER, outputPath=OUTPUT_DIR)