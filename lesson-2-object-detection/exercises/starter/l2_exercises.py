# ---------------------------------------------------------------------
# Exercises from lesson 2 (object detection)
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
import open3d as o3d
import math
import numpy as np
import zlib

import matplotlib
matplotlib.use('wxagg') # change backend so that figure maximizing works on Mac as well     
import matplotlib.pyplot as plt

# Exercise C2-4-6 : Plotting the precision-recall curve
def plot_precision_recall(): 

    # Please note: this function assumes that you have pre-computed the precions/recall value pairs from the test sequence
    #              by subsequently setting the variable configs.conf_thresh to the values 0.1 ... 0.9 and noted down the results.
    
    # Please create a 2d scatter plot of all precision/recall pairs
    import matplotlib.pyplot as plt
    P = [0.97, 0.94, 0.93, 0.92, 0.915, 0.91, 0.89, 0.87, 0.82]
    R = [0.738, 0.738, 0.743, 0.746, 0.746, 0.747, 0.748, 0.752, 0.754]
    plt.scatter(R, P)   
    plt.show()


# Exercise C2-3-4 : Compute precision and recall
def compute_precision_recall(det_performance_all, conf_thresh=0.5):

    if len(det_performance_all)==0 :
        print("no detections for conf_thresh = " + str(conf_thresh))
        return
    
    # extract the total number of positives, true positives, false negatives and false positives
    # format of det_performance_all is [ious, center_devs, pos_negs]
    pos_negs = [i[2] for i in det_performance_all]
    true_positives = np.sum([i[1] for i in pos_negs])
    false_positives = np.sum([i[3] for i in pos_negs])
    false_negatives = np.sum([i[2] for i in pos_negs])

    print("TP = " + str(true_positives) + ", FP = " + str(false_positives) + ", FN = " + str(false_negatives))
    
    #compute precision
    precision = true_positives/(true_positives+false_positives)
    
    #compute recall 
    recall = true_positives/(true_positives+false_negatives)

    print("precision = " + str(precision) + ", recall = " + str(recall) + ", conf_thres = " + str(conf_thresh) + "\n")    
    



# Exercise C2-3-2 : Transform metric point coordinates to BEV space
def pcl_to_bev(lidar_pcl, configs, vis=True):

    # compute bev-map discretization by dividing x-range by the bev-image height
    bev_discret = (configs.lim_x[1]-configs.lim_x[0])/configs.bev_height

    # create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates 
    # bev_discret is meters/pixel so divide metric x-coordinates by bev_discret to get pixel value for each coordinate
    # np.int_ is used to change the float values in the pcl to 8 bit integer pixel values (range: 0 to 255)
    lidar_pcl_copy = np.copy(lidar_pcl)
    lidar_pcl_copy[:, 0] = np.int_(np.floor(lidar_pcl_copy[:, 0]/bev_discret))
    
    # transform all metrix y-coordinates as well but center the foward-facing x-axis on the middle of the image
    lidar_pcl_copy[:, 1] = np.int_(np.floor(lidar_pcl_copy[:, 1]/bev_discret)+(configs.bev_width+1)/2)
    
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    # changing negative coordinate in the BEV space to unsigned integers results in flipping negative values
    # to 255 i.e. very bright values so we shift the ground plane up to avoid this
    lidar_pcl_copy[:, 2] = lidar_pcl_copy[:, 2] - configs.lim_z[0]
    
    # re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then by decreasing height (-z)
    # np.lexsort returns indices of the orignal array in the sorted order
    idx_height = np.lexsort((-lidar_pcl_copy[:,2], lidar_pcl_copy[:, 1], lidar_pcl_copy[:, 0]))
    lidar_pcl_hei = lidar_pcl_copy[idx_height]
    
    # extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    # np.unique returns sorted unique values, indices of the first occurrences of the unique values in the original array
    u , idx_sort_height = np.unique(lidar_pcl_hei[:,0:2], axis=0, return_index=True)
    lidar_pcl_hei = lidar_pcl_hei[idx_sort_height]
    
    # create a height map with height and width assigned in configs
    # assign the height value of each unique entry in lidar_top_pcl to the height map and 
    # make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    height_map = np.zeros((configs.bev_height+1, configs.bev_width+1))
    height_map[np.int_(lidar_pcl_hei[:, 0]), np.int_(lidar_pcl_hei[:, 1])] = lidar_pcl_hei[:, 2]/float(np.abs(configs.lim_z[1]-configs.lim_z[0]))
    
    # sort out highly reflective points with > 1.0 intensity and set them to 1.0
    lidar_pcl_copy[lidar_pcl_copy[:,3]>1.0, 3] = 1.0
    
    # sort points such that in case of identical BEV grid coordinates, the points in each grid cell are arranged based on their intensity
    idx_intensity = np.lexsort((-lidar_pcl_copy[:, 3], lidar_pcl_copy[:, 1], lidar_pcl_copy[:, 0]))
    lidar_pcl_inten = lidar_pcl_copy[idx_intensity]
    
    # only keep one point per grid cell
    _ , idx_sort_intensity = np.unique(lidar_pcl_inten[:,0:2], axis=0, return_index=True)
    lidar_pcl_inten = lidar_pcl_inten[idx_sort_intensity]

    # create the intensity map
    intensity_map = np.zeros((configs.bev_height+1, configs.bev_width+1))
    intensity_map[np.int_(lidar_pcl_inten[:, 0]), np.int_(lidar_pcl_inten[:, 1])] = lidar_pcl_inten[:, 3]/(np.amax(lidar_pcl_inten[:, 3]-np.amin(lidar_pcl_inten[:, 3])))
    
    # visualize height map
    if vis:
        img_height = height_map * 256
        img_height = img_height.astype(np.uint8)
        while (1):
            cv2.imshow('img_intensity', img_height)
            if cv2.waitKey(10) & 0xFF == 27:
                break
        cv2.destroyAllWindows()
    
    # visualize intensity map
    # if vis:
    #     img_intensity = intensity_map * 256
    #     img_intensity = img_intensity.astype(np.uint8)
    #     while (1):
    #         cv2.imshow('img_intensity', img_intensity)
    #         if cv2.waitKey(10) & 0xFF == 27:
    #             break
    #     cv2.destroyAllWindows()
    
    