# ---------------------------------------------------------------------
# Exercises from lesson 1 (lidar)
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Starter Code
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

from PIL import Image
import io
import sys
import os
import cv2
import numpy as np
import zlib

## Add current working directory to path
sys.path.append(os.getcwd())

## Waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2


# Exercise C1-5-5 : Visualize intensity channel
def vis_intensity_channel(frame, lidar_name):

    print("Exercise C1-5-5")
    # extract range image from frame
    lidar = [obj for obj in frame.lasers if obj.name==lidar_name][0]
    if len(lidar.ri_return1.range_image_compressed) > 0:
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
    ri[ri<0] = 0.0

    # map value range to 8bit
    ri_range = ri[:, :, 1] # [range, intensity, elongation, is_in_no_label_zone]
    # multiple entire range with half of max value to contrast adjust, else you will only see bright spots
    # normalize adjusted range and map to grayscale
    ri_range = np.amax(ri_range)/2 * ri_range * 255 /(np.amax(ri_range)-np.amin(ri_range))
    img_range = ri_range.astype(np.uint8)
    
    # focus on +/- 45° around the image center
    deg45 = int(img_range.shape[1]/8)
    img_center = int(img_range.shape[1]/2)
    # select all 64 rows, and center-45 & center+45 for columns
    img_range = img_range[:, img_center-deg45:img_center+deg45]

    cv2.imshow('intensity visulization', img_range) # cv2.imshow(window name, image_name)
    cv2.waitKey(0)

# Exercise C1-5-2 : Compute pitch angle resolution
def print_pitch_resolution(frame, lidar_name):

    print("Exercise C1-5-2")
    # load range image
    lidar = [obj for obj in frame.lasers if obj.name == lidar_name][0]
    if len(lidar.ri_return1.range_image_compressed) > 0: # use first response
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
   
    # compute vertical field-of-view from lidar calibration
    calib_lidar = [data for data in frame.context.laser_calibrations if data.name==lidar_name][0]
    vfov = calib_lidar.beam_inclination_max - calib_lidar.beam_inclination_min
    
    #vfov in degrees
    vfov_deg = vfov*180/np.pi

    # compute pitch resolution and convert it to angular minutes
    pitch_deg = vfov_deg/ri.shape[0]
    pitch_ang_min = pitch_deg*60
    
    print(pitch_ang_min)
    


# Exercise C1-3-1 : print no. of vehicles
def print_no_of_vehicles(frame):

    print("Exercise C1-3-1")    

    # find out the number of labeled vehicles in the given frame
    # Hint: inspect the data structure frame.laser_labels
    num_vehicles = 0
    for label_type in frame.laser_labels:
        #if label_type.type == 1 <-- hardcoded value for vehicle type
        if label_type.type == label_type.TYPE_VEHICLE:
            num_vehicles += 1
            
    print("number of labeled vehicles in current frame = " + str(num_vehicles))