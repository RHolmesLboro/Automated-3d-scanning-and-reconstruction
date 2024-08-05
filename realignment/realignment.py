import numpy as np
import logging, os, multiprocessing, time
from scipy.spatial.transform import Rotation as R

starttime = time.time()

format = "%(asctime)s.%(msecs)03d: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%Y-%m-%d, %H:%M:%S", filename="transformation.log", encoding="utf-8")

tcp = []
input_folder = r"C:\Users\ascii\Downloads\Project\translate\Data\original"
output_folder = r"C:\Users\ascii\Downloads\Project\translate\Data\output"
with open(r"C:\Users\ascii\Downloads\Project\translate\Data\TCP.csv", "r") as f:
    for line in f:
        line = line.strip().split(",")
        line = [float(coord) for coord in line]
        tcp.append(line)

# Ensure the output folder exists
if not os.path.exists(output_folder):
    os.makedirs(output_folder)
    logging.info("Output folder created")

def translate(filename, j):
    logging.info(f"Processing {filename} in process {j+1}")
    inputcloud = []
    rot = R.from_quat([tcp[j][3], tcp[j][4], tcp[j][5], tcp[j][6]]).as_euler("xyz", degrees=True)
    rot = R.from_euler("xyz", [rot[0], rot[1], rot[2]], degrees=True).as_matrix()
    translate = np.array([tcp[j][0]*1000, tcp[j][1]*1000, tcp[j][2]*1000])
    
    EEHSC = np.array([[-1.30813346e-01, -2.11620050e-03,  9.91404756e-01, -6.32019743e+01],
 [-9.91387158e-01, -6.04979635e-03, -1.30823938e-01, -9.50719044e+01],
 [ 6.27464656e-03, -9.99979461e-01, -1.30657989e-03, -3.54822303e+01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    with open(os.path.join(input_folder, filename), "r") as f:
        for line in f:
            line = line.strip().split()
            line = [float(coord) for coord in line]
            inputcloud.append(line)
    inputcloud = np.array(inputcloud).T  # Transpose for correct matrix multiplication
    inputcloud = inputcloud.T
    inputcloud += EEHSC[:3,3] #Translate CC
    logging.info(f"Translated {filename} according to CC in {j+1}")
    inputcloud = inputcloud.T
    inputcloud = np.matmul(EEHSC[:3,:3], inputcloud) #Rotate CC
    logging.info(f"Rotated {filename} around CC in {j+1}")
    inputcloud = np.matmul(rot, inputcloud).T #Rotate TCP 
    logging.info(f"Rotated {filename} around TCP in {j+1}") 
    inputcloud += translate #Translate TCP
    logging.info(f"Translated {filename} according to TCP in {j+1}")
    np.savetxt(os.path.join(output_folder, filename), inputcloud, fmt="%.6f", delimiter=" ")
    logging.info(f"Saved {filename} in thread {j+1}")

if __name__ == "__main__":
    processes = []

    for i, filename in enumerate(os.listdir(input_folder)):
        if filename.endswith(".asc"):
            logging.info(f"Processing file: {filename}")
            p = multiprocessing.Process(target=translate, args=(filename, i))
            processes.append(p)
    
    for p in processes:
        p.start()
        logging.info(f"Process {p.pid} started")
    for p in processes:
        p.join()

    logging.info(f"Total time: {time.time() - starttime:.2f} seconds")
