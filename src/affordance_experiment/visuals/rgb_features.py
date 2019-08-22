""" Extract 2D visual features from RGB image
This program calculates the 2D visual features of an object and save
the results to a CSV file.
1. Segment object from background using color filter
2. Extract object using connected component
3. Calculate features
4. Save to CSV file
"""
# Author: Yik Lung Pang

import cv2
import numpy as np
import operator
import copy
import os
import csv
import sys
from matplotlib import pyplot as plt
from math import pi


DEBUG_MODE = False

def main():
    """Extract 2D visual features from RGB image and save to CSV file

    """

    global DEBUG_MODE
    if len(sys.argv) == 2 and sys.argv[1] == '-d':
        DEBUG_MODE = True

    # List of objects to be processed
    object_list = ['stick', 'l_stick', 'bone', 'umbrella', 'fork', 'cube', 'sphere', 'cylinder']
    # List of tools
    tool_list = ['stick', 'l_stick', 'bone', 'umbrella', 'fork']

    # Open csv to save results
    script_dir = os.path.dirname(os.path.realpath(__file__))
    pos = script_dir.find('/visuals')
    feature_file_path = str(script_dir[:pos]+'/bn/features/rgb_features.csv')

    with open(feature_file_path, 'w') as feature_file:
        # Write header line
        header = ["object_name", "circleness", "squareness", "symmetry", "convexity", "eccentricity"]
        data_writer = csv.writer(feature_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
        data_writer.writerow(header)

        # Compute and save 2D features for each object
        for object_name in object_list:
            is_tool = False
            if object_name in tool_list:
                is_tool = True
            computeFeatures(object_name, is_tool, data_writer)


def computeFeatures(object_name, is_tool, data_writer):
    """Compute the 2D features and write to CSV

    Parameters
    ----------
    object_name : str
        Name of the object
    is_tool : boolean
        Is the object a tool
    data_writer : writer
        Data writter for writing to CSV

    """

    # Read RGB image
    script_dir = os.path.dirname(os.path.realpath(__file__))
    img = cv2.imread(script_dir+'/data/'+object_name+'0000.jpg', 1)
    # OpenCV reads img in BGR
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)  

    # In OpenCV HSV range is (0-179, 0-255, 0-255)
    # Use color filter to extract object
    range_light = (104, 204, 70)
    range_dark = (126, 255, 160)
    mask = cv2.inRange(hsv_img, range_light, range_dark)
    seg_img = cv2.bitwise_and(img, img, mask=mask)  

    # Get binary image
    seg_img = cv2.cvtColor(seg_img, cv2.COLOR_HSV2RGB)
    seg_img_gray = cv2.cvtColor(seg_img, cv2.COLOR_RGB2GRAY)
    ret, seg_img_binary = cv2.threshold(seg_img_gray, 50, 255, cv2.THRESH_BINARY)   

    if DEBUG_MODE:
        # Display progress
        titles = ['Original Image', 'Segmented Image']
        images = [img, seg_img_gray]
        for i in xrange(2):
            plt.subplot(2, 3, i+1)
            plt.imshow(images[i], 'gray')
            plt.xlim(300, 400)
            plt.ylim(450, 240)
            plt.title(titles[i])    

    # Extracting main object using connected components
    ret, labels = cv2.connectedComponents(seg_img_binary, connectivity=8)
    # Count the components and the area
    unique, counts = np.unique(labels, return_counts=True)
    # Remove background
    bg_index = np.where(unique == 0)
    unique = np.delete(unique, bg_index)
    counts = np.delete(counts, bg_index)    

    # Label of main object
    max_label = unique[counts.argmax()]

    # Only retain the main object
    seg_img_final = copy.deepcopy(seg_img_gray)
    seg_img_final[labels!=max_label] = 0
    seg_img_final[labels==max_label] = 255

    # Get bounding box
    min_x, max_x, min_y, max_y = getBoundingBox(seg_img_final)
    # Remove handle of tools
    if is_tool:
        seg_img_final = removeHandle(seg_img_final, min_y, max_y)
        min_x, max_x, min_y, max_y = getBoundingBox(seg_img_final)
    # Find contours
    cnt = getContours(seg_img_final)

    if DEBUG_MODE:
        # Display bounding box
        bb_copy = copy.deepcopy(seg_img_final)
        bb_copy = cv2.cvtColor(bb_copy, cv2.COLOR_GRAY2BGR) 
        cv2.line(bb_copy, (min_x,min_y), (max_x,min_y),[0,255,0],2)
        cv2.line(bb_copy, (min_x,max_y), (max_x,max_y),[0,255,0],2)
        cv2.line(bb_copy, (min_x,min_y), (min_x,max_y),[0,255,0],2)
        cv2.line(bb_copy, (max_x,min_y), (max_x,max_y),[0,255,0],2)
        plt.subplot(2, 3, 4)
        plt.imshow(bb_copy)
        plt.title('Bounding Box')
        plt.xlim(300, 400)
        plt.ylim(450, 240)

        # Display contours
        c_copy = copy.deepcopy(seg_img_final)
        c_copy = cv2.cvtColor(c_copy, cv2.COLOR_GRAY2BGR) 
        cv2.drawContours(c_copy, [cnt], 0, (0,255,0), 2)
        plt.subplot(2, 3, 5)
        plt.imshow(c_copy)
        plt.title('Contours')
        plt.xlim(300, 400)
        plt.ylim(450, 240)

    # Compute features
    circleness = computeCircleness(seg_img_final, cnt)
    squareness = computeSquareness(seg_img_final, min_x, max_x, min_y, max_y)
    symmetry = computeSymmetry(seg_img_final, min_x, max_x, min_y, max_y)
    convexity = computeConvexity(seg_img_final, cnt)
    eccentricity = computeEccentricity(cnt)
    # Write to CSV
    data_writer.writerow([object_name, circleness, squareness, symmetry, convexity, eccentricity])
    print('Processed '+str(object_name))

    if DEBUG_MODE:
        # Show diagrams
        plt.show()
    

def getBoundingBox(seg_img):
    """Get bounding box by searching for min and max of each dimension

    Parameters
    ----------
    seg_img : nparray
        Segmented image of the object

    Return
    ----------
    min_x : int
        Minimum x value of bounding box
    max_x : int
        Maximum x value of bounding box
    min_y : int
        Minimum y value of bounding box
    max_y : int
        Maximum y value of bounding box

    """

    # Search through all points for min and max of each dimension
    min_x = seg_img.shape[1]
    max_x = 0
    min_y = seg_img.shape[0]
    max_y = 0
    for y in xrange(seg_img.shape[0]):
        for x in xrange(seg_img.shape[1]):
            if seg_img[y,x] == 255:
                if x < min_x : min_x = x
                if x > max_x : max_x = x
                if y < min_y : min_y = y
                if y > max_y : max_y = y

    return min_x, max_x, min_y, max_y


def getContours(seg_img):
    """Get contours of object

    Parameters
    ----------
    seg_img : nparray
        Segmented image of the object

    Return
    ----------
    cnt : nparray
        Array of points on the contours

    """

    # Get binary image
    ret, thresh = cv2.threshold(seg_img,127,255,cv2.THRESH_BINARY)
    # Find contours
    _,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    cnt = contours[0]

    return cnt


def removeHandle(seg_img, min_y, max_y):
    """Get contours of object

    Parameters
    ----------
    seg_img : nparray
        Segmented image of the object
    min_y : int
        Minimum y value of bounding box
    max_y : int
        Maximum y value of bounding box

    Return
    ----------
    seg_img : nparray
        Image of object without handle

    """

    # Remove upper part of the bounding box
    for y in xrange(seg_img.shape[0]):
        if y < ((max_y-min_y)/2.0)+min_y and y >= min_y:
            for x in xrange(seg_img.shape[1]):
                seg_img[y,x] = 0

    if DEBUG_MODE:
        # Display object without handle
        plt.subplot(2, 3, 3)
        plt.imshow(seg_img, 'gray')
        plt.title('Removed Handle')
        plt.xlim(300, 400)
        plt.ylim(450, 240)

    return seg_img


def computeSquareness(seg_img, min_x, max_x, min_y, max_y):
    """Compute squareness
    Ratio between area of object and area of minimum bounding box

    Parameters
    ----------
    seg_img : nparray
        Segmented image of the object
    min_x : int
        Minimum x value of bounding box
    max_x : int
        Maximum x value of bounding box
    min_y : int
        Minimum y value of bounding box
    max_y : int
        Maximum y value of bounding box

    Return
    ----------
    squareness : float
        Squareness measure of the object

    """

    return float(np.sum(seg_img)/255.0)/(float(max_x-min_x+1)*float(max_y-min_y+1))


def computeCircleness(seg_img, cnt):
    """Compute circleness
    Ratio between area of object and area of minimum bounding circle

    Parameters
    ----------
    seg_img : nparray
        Segmented image of the object
    cnt : nparray
        Array of points on the contours

    Return
    ----------
    circleness : float
        Circleness measure of the object

    """

    # Compute radius of minimum bounding circle
    _,radius = cv2.minEnclosingCircle(cnt)

    return (float(np.sum(seg_img))/255.0)/(pi*(radius**2.0))

def computeSymmetry(seg_img, min_x, max_x, min_y, max_y):
    """Compute symmetry
    Reflect the image and calculate the overlap
    Symmetry is the average overlap of the x and y dimensions

    Parameters
    ----------
    seg_img : nparray
        Segmented image of the object
    min_x : int
        Minimum x value of bounding box
    max_x : int
        Maximum x value of bounding box
    min_y : int
        Minimum y value of bounding box
    max_y : int
        Maximum y value of bounding box

    Return
    ----------
    Symmetry : float
        Symmetry measure of the object

    """

    # Get image in bounding box and flip at mid point
    seg_img = seg_img[min_y:max_y+1, min_x:max_x+1]
    seg_img_flipx = np.fliplr(seg_img)
    seg_img_flipy = np.flipud(seg_img)

    # Obtain overlaping pixels
    overlap_lr = cv2.bitwise_and(seg_img, seg_img_flipx)
    overlap_ud = cv2.bitwise_and(seg_img, seg_img_flipy)

    # Calculate percentage of overlap
    symmetry_lr = float(np.sum(overlap_lr))/float(np.sum(seg_img))
    symmetry_ud = float(np.sum(overlap_ud))/float(np.sum(seg_img))

    return (symmetry_lr+symmetry_ud)/2.0


def computeConvexity(seg_img, cnt):
    """Compute convexity
    Ratio between perimeter of convex hull and perimeter of object contour

    Parameters
    ----------
    seg_img : nparray
        Segmented image of the object
    cnt : nparray
        Array of points on the contours

    Return
    ----------
    convexity : float
        Convexity measure of the object

    """

    # Perimeter of contour
    cnt_perimeter = cv2.arcLength(cnt,True)
    # Perimeter of convex hull
    hull = cv2.convexHull(cnt,returnPoints = True)
    hull_perimeter = cv2.arcLength(hull,True)
    
    if DEBUG_MODE:
        # Display convex hull
        seg_img = cv2.cvtColor(seg_img, cv2.COLOR_GRAY2BGR) 
        cv2.drawContours(seg_img, [hull], 0, (0,255,0), 2)
        plt.subplot(2, 3, 6)
        plt.imshow(seg_img)
        plt.title('Convex Hull')
        plt.xlim(300, 400)
        plt.ylim(450, 240)

    return hull_perimeter/cnt_perimeter

def computeEccentricity(cnt):
    """Compute eccentricity
    Ratio between the minor and major axis of the best-fit ellipse

    Parameters
    ----------
    cnt : nparray
        Array of points on the contours

    Return
    ----------
    eccentricity : float
        Eccentricity measure of the object

    """

    # Compute minor and major axis of the best fit ellipse
    _, axes, _ = cv2.fitEllipse(cnt)

    return axes[0]/axes[1]

## Main function
if __name__ == "__main__":
    main()