import laspy
import numpy as np
import os
import subprocess
import time
import configparser
import argparse

def validateArguments():
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(prog="txt2Potree",description='Convert point clouds from text file to Potree.')

    # Add arguments to the parser
    parser.add_argument('-i', '--input', type=str, required=True, help='Input file', dest="input")
    parser.add_argument('-s', '--seperator', type=str, required=False, default=',', help='Specify how data is seperated (default = \',\')', dest="seperator")
    parser.add_argument('-c', '--classification', type=int, required=False, default=4, help='Specify which is the classification column (default = 4)')

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
    return args.input, args.seperator, args.classification

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
    if data.shape[1] == cc:
        las.classification = data[:, cc-1].astype(int)

    # Write to LAS file
    LasFile = os.path.join(os.path.abspath(os.curdir), fileName)
    las.write(LasFile)
    
    return LasFile

def LasToPotree(lasFile, potreeConverterPath, outputPath):
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
    
    return outputFolder


if __name__ == "__main__":
    START_TIME = time.time()
    # Set working path to script location:
    os.chdir(os.path.split(__file__)[0])

     # Parse arguments
    INPUT_FILE, SEPERATOR, CLASSIFICATION_COLUMN = validateArguments()

    # Parse config.ini
    POTREE_CONVERTER, OUTPUT_DIR = parseConfig()

    # Get file name and extension
    fileName, ext = os.path.splitext(os.path.basename(INPUT_FILE))

    # Read input file
    print("\nLoading data from file...\n")
    points = np.loadtxt(INPUT_FILE, delimiter=SEPERATOR, dtype=np.float32)
    print(f"\nFile \"{os.path.basename(INPUT_FILE)}\" loaded. Time elapsed (s): {(time.time() - START_TIME):.4f}\n")

    # Convert to .las
    print("\nConverting data to .las file...\n")
    lasFile = txtToLas(points, CLASSIFICATION_COLUMN, fileName=fileName)
    print(f"\nSuccessfully generated .las file. Time elapsed (s): {(time.time() - START_TIME):.4f}\n")

    # Convert to Potree
    print("\nConverting .las file to Potree...\n")
    PotreePath = LasToPotree(lasFile=lasFile, potreeConverterPath=POTREE_CONVERTER, outputPath=OUTPUT_DIR)
    print(f"\nPotree files generated at {PotreePath}\n")

    print(f"Total time elapsed (s): {(time.time() - START_TIME):.4f}\n")