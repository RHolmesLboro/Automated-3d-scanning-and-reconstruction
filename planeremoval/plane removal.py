import numpy as np
import logging, os, multiprocessing
import pyransac3d as prs
from time import time

starttime = time()

# Setup logging
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")

# Define the paths
input_folder = r"C:\Users\ascii\Downloads\Project\planeremoval\Data\original"
output_folder = r"C:\Users\ascii\Downloads\Project\planeremoval\Data\planeremoved"

# Ensure the output folder exists
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

def loadandsegment(filename, input_filepath):
    # Read point cloud data from the file
    pointcloud2 = []
    with open(input_filepath, "r") as f:
        for line in f:
            line = line.split()
            line = [float(coord) for coord in line]
            pointcloud2.append(line)
    pointcloud = np.array(pointcloud2)
    pointcloud2.clear()
    logging.info(f"Read finished for file: {filename}")

    # Perform plane detection
    logging.info(f"Starting plane detection for file: {filename}")
    plane1 = prs.Plane()
    best_eq, best_inliers = plane1.fit(pointcloud, thresh=3) #modify thresh as needed
    logging.info(f"Plane detection complete for file: {filename}")

    # Remove points on plane and save the output
    logging.info(f"Removing points on plane for file: {filename}")
    outputcloud = np.delete(pointcloud, best_inliers, 0)
    output_filepath = os.path.join(output_folder, filename)
    np.savetxt(output_filepath, outputcloud, fmt="%.6f", delimiter=" ")
    logging.info(f"Points removed and file saved: {output_filepath}")

if __name__ == "__main__":
    processes = []

    # Process each file in the original folder
    for filename in os.listdir(input_folder):
        if filename.endswith(".asc"):  # Process only .asc files
            logging.info(f"Processing file: {filename}")
            
            # Construct full file path
            input_filepath = os.path.join(input_folder, filename)
            
            # Create a new process for each file
            p = multiprocessing.Process(target=loadandsegment, args=(filename, input_filepath))
            processes.append(p)

    logging.info("All processes created")

    for p in processes:
        p.start()

    logging.info("All processes running")

    for p in processes:
        p.join()

    logging.info("All processes finished")

    print(str(time()-starttime))
