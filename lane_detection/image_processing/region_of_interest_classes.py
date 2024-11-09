from abc import abstractmethod
import argparse
import math
import sys
import cv2
import time
import numpy as np 





def get_normal_sky_roi():
    return SkyRoi( 0.7)
from abc import abstractmethod
from abc import abstractmethod
class ROI:
    def __init__(self,use_inverted=False):
        self.width = None 
        self.height = None 
        self.mask = None
        self.inverted_mask = None
        self.use_inverted = use_inverted

    def config(self, height,width):
        self.width = width
        self.height = height
        self.mask = np.zeros((height, width), dtype=np.uint8)
        self.inverted_mask = None
        self.fill_poly_mask(self.get_roi_vertices())

    def fill_poly_mask(self, vertices):
        cv2.fillPoly(self.mask, [vertices], 255)
        self.inverted_mask = cv2.bitwise_not(self.mask)

    def get_mask(self):
        if self.use_inverted:
            return self.inverted_mask
        else:
            return self.mask
    
    @abstractmethod
    def get_roi_vertices(self):
        pass

class CarHoodRoi(ROI):
    def __init__(self,  
                 bottom_left, 
                 bottom_right, 
                 top_left_x, 
                 top_left_y, 
                 top_right_x, 
                 top_right_y):
        
        self.bottom_left = bottom_left
        self.bottom_right = bottom_right
        self.top_left_x = top_left_x
        self.top_left_y = top_left_y
        self.top_right_x = top_right_x
        self.top_right_y = top_right_y  
        super().__init__(True)
        
    def get_roi_vertices(self):
        self.roi_bottom_left = (int(self.width * self.bottom_left), 
                                self.height)
        self.roi_bottom_right = (int(self.width * self.bottom_right),
                                  self.height)
        self.roi_top_left = (int(self.width * self.top_left_x), 
                             int(self.height * self.top_left_y))
        self.roi_top_right = (int(self.width * self.top_right_x), 
                              int(self.height * self.top_right_y))
        return np.array([
                         self.roi_bottom_left, 
                         self.roi_top_left, 
                         self.roi_top_right, 
                         self.roi_bottom_right
                         ], 
                         np.int32)
    
@DeprecationWarning
class SkyRoi(ROI):
    def __init__(self, sky_top_y):
        super().__init__()
        self.sky_top_y = sky_top_y
        
    def get_roi_vertices(self):
        return np.array(
            [
                (0, int(self.sky_top_y * self.height)),
                (self.width, int(self.sky_top_y * self.height)),
                (self.width, self.height),
                (0, self.height)
            ], np.int32
        )

def get_normal_car_roi():
    return CarHoodRoi(0.15, 0.85, 0.35, 0.75, 0.65, 0.75)

class Roi_Handler:
    def __init__(self, 
                heigth,
                width,
                pixel_per_meter
                   ):
       
        self.height = heigth
        self.width = width
        self.rois = [get_normal_car_roi()]
        
        for roi in self.rois:
            roi.config(heigth,width)
        self.combied_mask = None
        self.combine_rois()



    def combine_rois(self):
        self.combied_mask = None
        for roi in self.rois:
            if self.combied_mask is None:
                self.combied_mask = roi.get_mask()
            else:
                  # Combine the filtered images using bitwise AND 
                self.combied_mask = cv2.bitwise_and(self.combied_mask,roi.get_mask())    
                

    def fiter_roi_and_candy(self, image):
        height, width = image.shape[:2]
        self.check_size_change(height, width)

        # Apply the inverted masks to the images
        edges = cv2.Canny(image, 100, 200)
        # Apply the non-black mask to the edges
        masked_edges = cv2.bitwise_and(edges, edges, mask=self.combied_mask)
        return masked_edges

    def get_mask(self,image):
        height, width = image.shape[:2]
        self.check_size_change(height, width)
        # Apply the inverted masks to the images
        return self.combied_mask

    def check_size_change(self, height,width):
        if height != self.height or width != self.width:
            self.height, self.width = height,width
            for roi in self.rois:
                roi.config(height,width)
            self.combine_rois()