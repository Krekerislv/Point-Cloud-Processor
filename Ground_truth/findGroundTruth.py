import numpy as np
from sklearn.neighbors import KDTree
from joblib import Parallel, delayed
import argparse
import os
import time
import traceback

def validateArguments():
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(description='Find matching points in .')

    # Add arguments to the parser
    parser.add_argument('-dr', '--raw', type=str, required=True, help='Raw data input')
    parser.add_argument('-dc', '--clean', type=str, required=True, help='Clean data input')
    parser.add_argument('-r', '--radius', type=float, required=True, help='radius in which to search for non-noise points')
    parser.add_argument('-sep', '--seperator', type=str, required=False, default=',', help='Specify how data is seperated (default = \',\')')
    parser.add_argument('-c', '--cores', type=int, required=False, default=4, help='Specify how many CPU cores to use (default = 4)')
    parser.add_argument('-s', '--save', type=str, required=False, help="Save ground truth file in specified file")

    # Parse the command line arguments
    args = parser.parse_args()

    # Check if input files exist
    if not os.path.isfile(args.raw):
        print(f"Raw data input file \"{args.raw}\" doesn't exist!")
        exit()

    if not os.path.isfile(args.clean):
        print(f"Clean data input file \"{args.clean}\" doesn't exist!")
        exit()
    
    if len(args.seperator) != 1:
        print(f"Invalid seperator: \"{args.seperator}\"")
        exit()

    # return arguments
    return args.raw, args.clean, args.radius, args.seperator, args.cores, args.save

def find_matching_points(raw_points, clean_points, radius):
    # Create an empty array to store the results
    results = np.zeros((raw_points.shape[0], 4))

    # Build KD-tree on clean_points
    tree = KDTree(clean_points)

    # Find neighbors within the radius for each raw point
    points_within_radius_indices = tree.query_radius(raw_points, radius)

    # Mark matching points with 1 in the fourth column
    for i, ind in enumerate(points_within_radius_indices):
        if len(ind) > 0:
            results[i, 3] = 1

    # Set 255 for non-matching points
    points_outside_radius_indices = np.where(results[:, 3] != 1)[0]
    results[points_outside_radius_indices, 3] = 255

    # Copy raw points to the result array
    results[:, :3] = raw_points

    return results


if __name__ == "__main__":
    START_TIME = time.time()

    # Get input arguments    
    raw_file, clean_file, radius, sep, cores, save_file = validateArguments()

    # Load raw data and clean data
    raw = np.loadtxt(raw_file, delimiter=sep, dtype=np.float32)
    clean = np.loadtxt(clean_file, delimiter=sep, dtype=np.float32)
    

    print(f"\nFiles \"{os.path.basename(raw_file)}\" and \"{os.path.basename(clean_file)}\" loaded. Time elapsed (s): {(time.time() - START_TIME):.4f}\n")
    
    # Split raw data into chunks and process in parallel
    results = Parallel(n_jobs=cores)(
        delayed(find_matching_points)(chunk, clean, radius)
        for chunk in np.array_split(raw, cores)
    )

    # Combine the results from different chunks
    results = np.concatenate(results)

    # Display results
    print("\n==================================================")
    print(f"|| Time elapsed (s):\t\t{(time.time() - START_TIME):.4f}\t\t||")
    print(f"|| Raw point count:\t\t{raw.shape[0]}\t\t||")
    print(f"|| Clean point count:\t\t{clean.shape[0]}\t\t||")
    print(f"|| Radius:\t\t\t{radius}\t\t||")
    print(f"|| Matching point count:\t{results[results[: , 3] == 1].shape[0]}\t\t||")
    print(f"|| Noise point count:\t\t{results[results[: , 3] == 255].shape[0]}\t\t||")
    print(f"|| Actual noise point count:\t{raw.shape[0] - clean.shape[0]}\t\t||")
    print(f"|| Error:\t\t\t{abs(raw.shape[0] - clean.shape[0] - results[results[: , 3] == 255].shape[0])}\t\t||")
    print("==================================================\n")
    # Save results
    if save_file != None:
        try:
            print("\nSaving ground truth...\n")

            file_name, ext = os.path.splitext(os.path.basename(raw_file))
            path = os.path.join(save_file)

            np.savetxt(path, results, delimiter=sep, fmt='%.2f')

            print(f"\nGround truth file saved at \"{path}\"\n")
        except Exception:
            print(f"{traceback.format_exec()}\n")

    print(f"\nTotal program execution time: {(time.time() - START_TIME):.4f} seconds.\n")
