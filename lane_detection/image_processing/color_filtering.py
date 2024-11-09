import cv2
import numpy as np


class ColorRange:
    def __init__(self, hue_down, sat_down, val_down, hue_up, sat_up, val_up):
        self.hue_down = hue_down
        self.sat_down = sat_down
        self.val_down = val_down
        self.hue_up = hue_up
        self.sat_up = sat_up
        self.val_up = val_up


class Color_Filter_Handler:
    def __init__(self, color_ranges,kernel_fill_size=(3,3),kernel_erode_size=(3,3) ,kernel_size=(9,9)):
        self.mask = None 
        self.color_ranges = color_ranges
        self.kernel_fill_size = kernel_fill_size
        self.kernel_erode_size = kernel_erode_size
        self.kernel_size = kernel_size

    def imageToRgb(img_bgr):
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        return img_rgb

    # Convert BGR image to HSV
    def image_to_hsv(self,img_brg):
        hsv_image = cv2.cvtColor(img_brg, cv2.COLOR_BGR2HSV)
        return hsv_image

    def morphology_filter(self):
        # Creating kernel
        kernel = np.ones(self.kernel_size, np.uint8)
        # Apply closing operation to the mask
        self.mask =  cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        return self.mask

    def filter_color(self, img):
        img_hsv = self.image_to_hsv(img)
        self.mask = None
        for color_range in self.color_ranges:
            low = np.array([color_range.hue_down, color_range.sat_down, color_range.val_down])
            up = np.array([color_range.hue_up, color_range.sat_up, color_range.val_up])
            mask_1 = cv2.inRange(img_hsv, low, up)
            if self.mask is None:
                self.mask = mask_1
            else:
                self.mask = cv2.bitwise_or(self.mask, mask_1)  
        return self.mask



    def filter_dilate(self):
        _, thresh = cv2.threshold(self.mask, 1, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones(self.kernel_erode_size, np.uint8)
        thresh = cv2.erode(thresh, kernel)

        kernel = np.ones(self.kernel_fill_size, np.uint8)
        # Dilation is a morphological operation that expands the white 
        # regions in a binary image while filling gaps.
        thresh = cv2.dilate(thresh, kernel)       

        return thresh


    def adaptive_thresholding_segmentation(self, binary_image):
        """
        Apply adaptive thresholding-based segmentation to detect lane lines.

        This function processes a binary image to segment and detect lane lines based on the adaptive thresholding method.

        Args:
            binary_image: Binary image containing lane information.

        Returns:
            ndarray: Filtered image with detected lane regions.

        Source:
        Xin Yang and Houyu Yu, "Lane Line Detection Based on Dynamic ROI and 
        Adaptive Thresholding Segmentation."
        https://dl.acm.org/doi/pdf/10.1145/3573428.3573524
        """
        # Calculate the number of image rows
        num_rows = binary_image.shape[0]

        # Define thresholds
        vertical_distance_threshold = (1 / 500) * num_rows
        width_threshold_percentage = 0.2  # Adjust this value as needed

        # Find connected components in the binary image
        _, connected_components = cv2.connectedComponents(binary_image)

        # Iterate through the extracted connected components
        filtered_regions = []
        for label in range(1, np.max(connected_components) + 1):
            region_mask = (connected_components == label)
            y_coordinates = np.nonzero(region_mask)[0]
            y_max = np.max(y_coordinates)
            y_min = np.min(y_coordinates)
            y_distance = y_max - y_min

            # Check the vertical distance condition
            if y_distance > vertical_distance_threshold:
                # Calculate the pixel count in each row
                row_pixel_counts = np.sum(region_mask, axis=1)
                num_non_lane_rows = np.sum(row_pixel_counts > width_threshold_percentage 
                                           * binary_image.shape[1])

                # Check the width condition
                if num_non_lane_rows <= width_threshold_percentage * num_rows:
                    filtered_regions.append(region_mask)

        # Create an empty output image
        filtered_image = np.zeros_like(binary_image)

        # Set the filtered regions in the output image to 255
        for idx, region_mask in enumerate(filtered_regions):
            filtered_image[region_mask] = 255

        return filtered_image

    def get_tresh_image(self,thresh):
        return cv2.merge((thresh, thresh, thresh))
         