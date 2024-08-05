import cv2, glob, logging
import numpy as np
from scipy.spatial.transform import Rotation

#setup logging formatting
format = "%(asctime)s.%(msecs)03d: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%Y-%m-%d, %H:%M:%S", filename="handeyecal.log", encoding="utf-8")

#load wrist positions
def loadcsvvalues(projectpath, csvpath):
    logging.info("loading wrist positions")
    data = open(projectpath+csvpath).read().split("\n") #loads data into "data" as a list split on every line
    for i in range(len(data)): #itterate through each line
        data[i] = data[i].split(",") #split each list into lists
        data[i] = [float(j) for j in data[i]] #convert each value in the line from a string to a float.
    transforms = []

    for row in data: #itterate through data row by row
        t = row[0:3] #t is the first 3 values for each row (xyz vector base to wrist)
        q = row[3:7] #quat is the last 4 values from each row
        R = Rotation.from_quat(q).as_matrix() #rotation matrix R created from quat
        t = [i*1000 for i in t] #convert vector from m to mm
        H = np.eye(4) #create T as a 4x4 identity matrix
        H[:3,:3] = R #top left of T becomes rotation matrix
        H[:3,3] = t #right 3 positions become column vector
        transforms.append(H) #adds new 4x4 H matrix to transforms list

    return transforms #returns transforms list of H matracies to calling location

def find_corners(images, patternsize):
    corners = []
    imgindex = []
    i = 0
    logging.info("Finding corners")
    for image in images: #itterates through images as image
        found, con = cv2.findChessboardCorners(image, patternsize) #found returns true or false depending on if corners are all found, con is a list of the coordinates of the corners
        if found:
            corners.append(con) #adds corners to list corners for each image
            imgindex.append(i) #adds image position to index
        else:
            logging.error(f"chessboard not found in image {i+1}") #raise an error if chessboard not found but continue
        i+=1 #itterate current position

    logging.info(f"Found chessboard in {len(imgindex)} out of {len(images)} images")
    return corners, imgindex #returns corners list and index list to calling location

def calcreprojection(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    terror = 0
    npoints = 0
    errors = []
    for i in range(len(objpoints)): #itterate through for every point on the chessboard
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist) #projects the expected positions of the points according to the calibration and saves them to projected
        projected = projected.reshape(-1, 1, 2) #reshapes the array of projected points to a tensor with as many rows as needed, 1 column wide, and 2 deep
        error = cv2.norm(imgpoints[i], projected, cv2.NORM_L2)/len(projected) #calculate the absolute total differnce norm and devide by the number of points to find the normal error for this image
        errors.append(error) #adds this error norm to the list of error norms
        terror += error #adds this error adverage to the total error adverage
        npoints += 1 #itterates npoints
        logging.info(f"image {i+1} error: {error}") #logs individual error
    logging.info(f"adverage error of {npoints} images: {terror/npoints}") #logs adverage error
    # return terror/npoints #returns adverage error to calling location (this is not normally needed)

def calcintrinsics(corners, index, patternsize, squaresize):
    objpoints = []
    for i in range(len(index)): #for every image where corners were found
        objp = np.zeros((patternsize[0]*patternsize[1], 3), np.float32) #creates a matrix of zeros that has a row per space on the grid and 3 columns wide with a datatype suitible for 32 bit floating point numbers
        objp[:,:2] = np.mgrid[0:patternsize[0], 0:patternsize[1]].T.reshape(-1,2)*squaresize #make objp an array with the coordinates of the object points for the intersections of the chessboard squares
        objpoints.append(objp) #add each array to objpoints (one array per image where points were found)
    _, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, corners, images[0].shape[:2], None, None) #sets mtx to the camera intrinsics matrix, dist to the distortion coeefitients, rvec to the rotation of the camera points relative to the real chessboard, and tvecs to the translation from the real positions to the observed positions
    calcreprojection(objpoints, corners, rvecs, tvecs, mtx, dist) #calculate reprojection
    return mtx, dist #return intrinsic matrix and distortion coefficients to calling location

def findcampos(corners, patternsize, squaresize, intrinsicmatrix, dist = None):
    objectpoints = np.zeros((patternsize[0]*patternsize[1], 3), np.float32) #creates a matrix of zeros that has a row per space on the grid and 3 columns wide with a datatype suitible for 32 bit floating point numbers
    objectpoints[:, :2] = np.mgrid[0:patternsize[0], 0:patternsize[1]].T.reshape(-1,2)*squaresize #make objp an array with the coordinates of the object points for the intersections of the chessboard squares
    rtarget2cam = []
    ttarget2cam = []
    i = 1
    for con in corners:#itterate through all the lists of corners
        _, rvec, tvec = cv2.solvePnP(objectpoints, con, intrinsicmatrix, dist) #finds rotation and translation of camera that gives the projection resulting in the object points
        i+=1#itterate i
        R, _ = cv2.Rodrigues(rvec) #R is the rotaion matrix from the target to the camera accoriding to the Rodrigues method
        rtarget2cam.append(R) #appends rotation matrix to rotation tensor
        ttarget2cam.append(tvec) #append translation vector to translation vector list
    return rtarget2cam, ttarget2cam

patternsize = (17, 24) #no grid squares
squaresize = 15 #square size in mm
projectpath = "data/inovo_polyga/" #path to where all the data is stored
imagefolder = "RGBImgs/" #path within project path where images are stored
csvpath = "tcp.csv"

#load images
imagefiles = sorted(glob.glob(projectpath+imagefolder+"*.bmp")) #returns a list of all files in the image folder with the .bmp extention
logging.info(f"files found: {imagefiles}") #shows all the file names
images = [cv2.cvtColor(cv2.imread(f), cv2.COLOR_RGB2GRAY) for f in imagefiles] #opens all images to a list of numpy arrays of grayscale values

base2wrist = loadcsvvalues(projectpath, csvpath) #loads wrist transforms to "base2wrist"
if len(images) != len(base2wrist): logging.error("number of wrist positions and number of images do not match")

corners, index = find_corners(images, patternsize) #sets corners and index to list of corner locations and index of the assosiated image
intrinsicmatrix, dist = calcintrinsics(corners, index, patternsize, squaresize) #calculate intrinsics matrix and save
np.savez(projectpath+"IntrinsicMatrix.npz", intrinsicmatrix) #save intrinsic matrix to npz file

rtarget2cam, ttarget2cam = findcampos(corners, patternsize, squaresize, intrinsicmatrix, dist) #creates rtarget2cam rotation tensor and ttarget2cam translation matrix (camera extrinsics)

rwrist2base = []
twrist2base = []

for T in base2wrist:
    rwrist2base.append(T[:3, :3]) #invert base2wrist rotation portion and adds to list of rotation matracies
    t = [i*-1 for i in T[:3,3]] #inverts all values of vector
    t = np.array(t).reshape((3, 1)) #reshapes vector to 3x1 matrix
    twrist2base.append(t) #adds to list of translation vectors

twrist2base = np.array(twrist2base)

for i in range(0,5):
    logging.info(f"Trying method {i}")
    try:
        rcam2wrist, tcam2wrist = cv2.calibrateHandEye(rwrist2base, twrist2base, rtarget2cam, ttarget2cam, method=i) #calculate rotation matrix and translation vector from camera to wrist
        Tcam2wrist = np.concatenate((np.concatenate((rcam2wrist, tcam2wrist), 1),[[0, 0, 0, 1]]),0) #joins rotation matrix and translation vector to form a 4x4 transformation matrix
        Twrist2cam = np.concatenate((np.concatenate((np.linalg.inv(rcam2wrist), tcam2wrist*-1),1), [[0, 0, 0, 1]]), 0) #joins inverse of translation and rotation to form the inverse 4x4 matrix
        np.savez(projectpath+f"FinalTransforms/Twrist2cam_method_{i}.npz", Twrist2cam) #saves an an npz
        np.savez(projectpath+f"FinalTransforms/Tcam2wrist_method_{i}.npz", Tcam2wrist) #saves an an npz
        logging.info(f"camera to wrist vector: {tcam2wrist.T}\ncamera to wrist rotation (euler): {Rotation.from_matrix(rcam2wrist).as_euler("xyz", degrees=True)}") #logs the output
        logging.info(f"wrist to camera vector: {(tcam2wrist*-1).T}\ncamera to wrist rotation (euler): {Rotation.from_matrix(np.linalg.inv(rcam2wrist)).as_euler("xyz", degrees=True)}") #logs the output
    except Exception as e:
        logging.error(f"Method {i} failed\nReason: {e}")

logging.shutdown()