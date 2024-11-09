import math
import cv2
import numpy as np
import bisect
from deprecated import deprecated
from skimage import morphology

class FilterMiddleLines:
    def __init__(self,
                 height,
                 width,
                 pixel_pro_meter,
                min_distance_between_lanes = 0.20,
                max_distance_between_lanes = 0.37, 
                min_length=0.25,
                max_length=0.35,
                max_aspect_ratio=20,
                max_line_fit_error=20,
                first_line_fit_error=10,
                max_width=200,
                min_winkel = -1.8,
                max_winkel = 1.8,
                max_allow_diffrence = 0.2,
                max_angle_difference = 0.4,
                max_diff_prediction =  0.4,
                devation_filter = False
                ): 
        self.min_distance_between_lanes = min_distance_between_lanes
        self.max_distance_between_lanes = max_distance_between_lanes

        self.max_diff_prediction = max_diff_prediction
        self.pixel_pro_meter = pixel_pro_meter
        self.height = height
        self.width = width
        self.max_allowed_distance_to_next = max_allow_diffrence
        self.first_line_fit_error = first_line_fit_error
                
        self.max_angle_difference = max_angle_difference 
        self.min_length = min_length
        self.max_length = max_length
        self.max_aspect_ratio = max_aspect_ratio
        self.max_line_fit_error = max_line_fit_error
        self.max_width = max_width
        self.min_angle = min_winkel
        self.max_angle = max_winkel
        self.devation_filter = devation_filter
        self.perfect_points = []

    def is_middle_lane(self,lane_contour):
        """
        Determine if a given lane contour represents a middle lane based on various criteria.

        Args:
            self: The object instance.
            lane_contour: A LaneContour object representing a detected lane contour.

        Returns:
            bool: True if the lane contour is considered a middle lane, False otherwise.
        """
        if lane_contour.deviation_angle > self.max_angle \
            or lane_contour.deviation_angle < self.min_angle:
            return False

        if lane_contour.lane_length < self.min_length: 
            return False
        if lane_contour.lane_length > self.max_length:
            return False

        if lane_contour.bottom_point[1] > self.height * 0.8:
            return False

        if self.devation_filter:
            if lane_contour.deviation_angle + 0.5 > self.max_angle or \
                lane_contour.deviation_angle -0.5 < self.min_angle:
                x = lane_contour.center[0][1]
                if x > self.width * 3/4:
                    return False
                if x < self.width * 1/4:
                    return False
                    

        if lane_contour.width > self.max_width:
            return False

        if lane_contour.line_fit_error > self.max_line_fit_error:
            return False
        return True

    def create_middle_lines(self,middle_line_countours,player_point):
        self.middle_line_countours = middle_line_countours
        for ele in self.middle_line_countours:
            distance = np.linalg.norm(player_point - ele.center)
            ele.distance = distance

        self.middle_line_countours.sort(key=lambda middle_line_countours: middle_line_countours.distance)
        ##print(f'pre filter THrough middle lines{len(middle_line_countours)}')
        return self.filter_through_middle_lines()
      


    def filter_through_middle_lines(self):
        """
        Filter a list of middle line contours based on distance and angle criteria.

        This function processes a list of middle line contours and filters them based on
        their deviation angle and distance from each other. Contours that do not meet
        the specified criteria are removed from the list.

        Args:
            self: The object instance.

        Returns:
            None: The filtered contours are stored back in the object's attribute.
        """

        # Check if there are any middle line contours to filter
        if len(self.middle_line_countours) < 1:
            return []

        # Define maximum distance and angle difference criteria
        max_distance = 200  
        max_angle_difference = self.max_angle_difference

        # Initialize variables for tracking the previous contour
        prev_point = self.middle_line_countours[0].center
        prev_angle = self.middle_line_countours[0].deviation_angle
        first_line_fit_error = self.middle_line_countours[0].line_fit_error

        if first_line_fit_error > self.first_line_fit_error:
            self.middle_line_countours = self.middle_line_countours[1:]
        #    #print(self.first_line_fit_error)
            return self.filter_through_middle_lines()
        # Check and filter the first contour based on angle and distance criteria
        if np.abs(prev_angle) > 1.3:
            self.middle_line_countours = self.middle_line_countours[1:]
        #    #print("2")
            return self.filter_through_middle_lines()

        if (self.height - prev_point[0][1]) > (self.height * 0.8):
            self.middle_line_countours = self.middle_line_countours[1:]
          #  #print("3")
            return self.filter_through_middle_lines()

        # Initialize a list to store filtered contours
        filtered_countours = [self.middle_line_countours[0]]

        # Iterate through the remaining contours
        for i in range(1, len(self.middle_line_countours)):
            current_contour = self.middle_line_countours[i]

            # Calculate the angle between the previous and current contour
            angle_betw_points = np.arctan2(prev_point[0][0] - current_contour.center[0][0],
                                        prev_point[0][1] - current_contour.center[0][1])

            # Calculate the distance between the previous and current contour
            distance = np.linalg.norm(prev_point - current_contour.center)/self.pixel_pro_meter

            # Calculate the angle difference between current and previous contour
            angle_diff = np.abs(current_contour.deviation_angle - prev_angle)
            

            if i > 1 and  distance < self.min_distance_between_lanes or distance > self.max_distance_between_lanes:
            #   #print(f' distance{distance}' )
               continue

            # where should the point be 
            perfect_point = np.array([
                        prev_point[0][0] + distance * math.cos(prev_angle),
                        prev_point[0][1] + distance * math.sin(prev_angle)
                    ])



            distance_between_centers = np.linalg.norm(current_contour.center - perfect_point) / self.pixel_pro_meter
            ##print(f"Distance between current_contour.center and perfect_point: {distance_between_centers}")
            
            if i > 1 and distance_between_centers > self.max_diff_prediction:
                continue

            # Check if the angle difference exceeds the maximum allowed
            if len(filtered_countours) > 1:
                if np.abs(current_contour.deviation_angle - np.abs(angle_betw_points)) \
                > max_angle_difference:
                    continue

            # Check if the distance and angle meet the criteria
            if distance <= max_distance:
                
                if np.abs(angle_diff) <= max_angle_difference:
                    filtered_countours.append(current_contour)
                    prev_point = current_contour.center
                    prev_angle = current_contour.deviation_angle
                    continue
                else:
                    continue
            else:
                continue

        
        if len(filtered_countours) == 1 and len(self.middle_line_countours) > 5:
            self.middle_line_countours = self.middle_line_countours[1:]
            alt_filtered_countours = self.filter_through_middle_lines()
            if len(alt_filtered_countours) > 2:
                return alt_filtered_countours

        return filtered_countours            




class LaneContourHandler:
    def __init__(self, 
                 thresh,
                 pixel_pro_meter,
                 allowed_distance_forward,
                 allowed_distance_sideways,
                 filter_middle_lines= [],
                 max_line_fit_error=40,
                 max_width=0,
                 min_width=0,
                 min_length=80,
                 tangens_filter=False
                 ):
        
        
        # new parameters 
        self.pixel_pro_meter = pixel_pro_meter
        self.allowed_distance_forward  = allowed_distance_forward
        self.allowed_distance_sideways  = allowed_distance_sideways



        # old parameters
        self.filter_bottom_value = True
        height, width = thresh.shape
        self.width = width
        self.height = height
        filter_middle_lines_obj = FilterMiddleLines(height, width,pixel_pro_meter)
        self.player_point = (width / 2, height * 0.85)
    
        self.thresh = thresh
        self.filter_middle_lines_obj = filter_middle_lines_obj

        #gray_thresh = thresh.astype(np.uint8)
        
        self.edges = cv2.Canny(thresh,40,255) 
        contours, hierarchy = cv2.findContours(self.edges, 
                                               cv2.RETR_EXTERNAL, 
                                               cv2.CHAIN_APPROX_SIMPLE)
        

        


        self.middle_line_countours = []


        self.split_contours = []  

        self.stop_line = []

        self.left_line_countours = []
        self.right_line_countours = []

        self.middle_points_debug = []
        self.tangens_filter = tangens_filter
        height, width = self.thresh.shape
        self.dx = 0.5

        self.max_middle_x = width
        self.min_middle_x = width

        self.middle_max_height = 0
        self.middle_min_height = height
        
        self.all_lane_obj = []

        self.possible_side_lines = []
        self.other_countours = []


        self.middle_line_points = []
        self.middle_lines = []
        self.right_lines_points = []
        self.left_lines_points = []


        for countour in contours:
            obj = LaneCountour(countour, pixel_pro_meter,height=height,width=width)
            if obj.cx == 0 and obj.cy == 0:
                continue     
            self.all_lane_obj.append(obj)
        

        ##print(f'start elemente die rausgefiltert sind  {len(self.prefiltered_lane_countours)}')
        # create a skeleton for every contour   
        if False:
            for contour_obj in self.prefiltered_lane_countours:
                contour = contour_obj.contour
                mask = np.zeros_like(self.thresh)
                cv2.drawContours(mask, [contour], 0, 255, 1)
                contour_obj.skeleton = morphology.skeletonize(mask)
       
        self.find_middle_lane_elements()
       # #print(f'pre filter THrough middle lines{len(self.middle_line_countours)}')
        self.middle_line_countours = self.filter_middle_lines_obj.create_middle_lines(self.middle_line_countours, self.player_point)
        
        #for ele in self.middle_line_countours:
        #    #print(f' {ele.line_fit_error} width {ele.width} , "length {ele.lane_length} contoureara {ele.contour_area}')
        self.filter_side_lane_elements()
        self.create_left_right_lanes()
    


    # Go through all pre filtered contours and find out if they are middle lane elements
    def find_middle_lane_elements(self):
        i = 0
        while i < len(self.all_lane_obj):
            ###print(i)
            if self.filter_middle_lines_obj.is_middle_lane(self.all_lane_obj[i]):
                self.middle_line_countours.append(self.all_lane_obj.pop(i))
            else :
                i = i + 1

    def filter_side_lane_elements(self):
        j = 0
        flag = False
        while j < len(self.all_lane_obj):
            
            if self.is_side_lane_element(self.all_lane_obj[j]):
                self.possible_side_lines.append(self.all_lane_obj.pop(j))
            elif(True):
                self.other_countours.append(self.all_lane_obj.pop(j))    
            elif (False) :  
                cont_obj = self.all_lane_obj.pop(j)
                if cont_obj.lane_length > 2:
                    contour = cont_obj.contour
                    contour_image = np.zeros_like(self.edges)
                    if flag:
                        break
                    flag = True
                    cv2.drawContours(contour_image, [contour], -1, (255), thickness=cv2.FILLED)
                    skeleton = cv2.ximgproc.thinning(contour_image, np.ones((50,50), np.uint8))


                    print(simplified_endpoints)
                    print(len(simplified_endpoints))

                    for i in range(len(simplified_endpoints) - 1):
                        start_point = simplified_endpoints[i]
                        end_point = simplified_endpoints[i + 1]
                        split_contour = contour_image[start_point:end_point]
                        split_contours, _ = cv2.findContours(split_contour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        self.split_contours.extend(split_contours)
            
            elif(False):
                cont_obj = self.all_lane_obj.pop(j) 
            
                # Split the contour into multiple parts
                epsilon = 0.02 * cv2.arcLength(cont_obj.contour, True)
                approx = cv2.approxPolyDP(cont_obj.contour, epsilon, True)

                # Process each part separately
                for i in range(1, len(approx)):
                    # Extract contour points of the specified segment
                    segment_points = approx[i-1:i+1].reshape(-1, 2)

                    # Calculate the bounding rectangle for the segment
                    rect = cv2.boundingRect(segment_points)

                    # Extract width and height of the bounding rectangle
                    width, height = rect[2], rect[3]

                    # Check if both width and height are greater than 4
                    if width > 4 and height > 4:
                        # Save the contour points
                        self.split_contours.append(segment_points)
            
            
            elif(True):
                cont_obj = self.all_lane_obj.pop(j)
                cont_obj.debug_message_side_line = "!!!!!!!!!!!!!!"
                

                
                if cont_obj.lane_length > 1:
                   # self.other_countours.append(cont_obj)
                    # Split the contour into multiple parts
                    epsilon = 0.002 * cv2.arcLength(cont_obj.contour, False)
                    approx = cv2.approxPolyDP(cont_obj.contour, epsilon, False)
                 #   print("length... " + str(len(approx)))
                    new_contour=[]
                    has_stop_line = False
                    index = None
                    stop_angle = None
                    min_point = None
                    max_point = None

                    cont_obj.debug_message_side_line = str(len(approx))
                    self.other_countours.append(cont_obj)

                    for i in range(1, len(approx)):
                        # Calculate the length of the approximated line
                      #  self.stop_line.append(approx[i-1:i+1])
                        length = cv2.norm(approx[i-1], approx[i])/ self.pixel_pro_meter
                     #   print(length)
                        if len(approx) > 4 and length > 0.23 and length < 0.28:
                           # print(length)
                          #  print("hello " + str(cont_obj.center))
                            has_stop_line = True
                            
                            #self.stop_line.append(approx[i-1:i+1])
                            #print(f"angle{stop_angle} + length{length}")

                            if (i > 2):
                                length = cv2.norm(approx[i-2], approx[i-1])/ self.pixel_pro_meter
                                stop_angle = np.arctan2(approx[i-1][0][1] - approx[i-2][0][1], approx[i-1][0][0] - approx[i-2][0][0])
                             #   print(f" prev angle{stop_angle} + length{length}")

                            if i < len(approx) - 1:
                                length = cv2.norm(approx[i], approx[i+1])/ self.pixel_pro_meter
                                stop_angle = np.arctan2(approx[i+1][0][1] - approx[i][0][1], approx[i+1][0][0] - approx[i][0][0])
                              #  print(f" next angle{stop_angle} + length{length}")
                            #self.stop_text.append("angle")
                            
                            index = i
                            break

                    
                       
                    if has_stop_line == True:
                        i = index
                        if len(approx) >= 4:

                           # prev_angle = np.arctan2(approx[index][0][1] - approx[index-1][0][1], approx[index][0][0] - approx[index-1][0][0])

                            if index > 1  and index < len(approx)-2 :
                                
                                prev_angle = np.arctan2(approx[index-1][0][1] - approx[index-2][0][1], approx[index-1][0][0] - approx[index-2][0][0])
                                
                                prev_length = cv2.norm(approx[i-2], approx[i-1])/ self.pixel_pro_meter

                                stop_angle = np.arctan2(approx[i][0][1] - approx[i-1][0][1], approx[i][0][0] - approx[i-1][0][0])
                                next_angle = np.arctan2(approx[i+1][0][1] - approx[i][0][1], approx[i+1][0][0] - approx[i][0][0])

                                next_length = cv2.norm(approx[i], approx[i+1])/ self.pixel_pro_meter

                                prev_diff = np.abs(prev_angle - stop_angle)
                                
                                next_diff = np.abs(prev_angle - next_angle)
                                print(f' length {prev_length } {next_length}')
                                print(f' Number one {prev_diff } {next_diff}')
                                
                                if np.abs(0- np.abs(stop_angle)) < 0.2 or np.abs( np.pi * 2 - np.abs(stop_angle)) < 0.2 or  np.abs( np.pi * 1 - np.abs(stop_angle)) < 0.2:

                                    if np.abs(prev_length - 0.036) < 0.015 or np.abs(next_length - 0.036) < 0.015 or np.abs(prev_length - 0.0833) < 0.015 or np.abs(prev_length - 0.0833) < 0.015:
                                        if prev_diff > 1.4 or next_diff > 1.4:
                                            self.stop_line.append(approx[i-1:i+1])
                                            continue
                                
                            elif index == 0 :

                                prev_angle = np.arctan2(approx[i][0][1] - approx[len(approx)-1 ][0][1], approx[i][0][0] - approx[len(approx) - 1][0][0])

                                stop_angle = np.arctan2(approx[i+1][0][1] - approx[i][0][1], approx[i+1][0][0] - approx[i][0][0])
                                next_angle = np.arctan2(approx[i+2][0][1] - approx[i+1][0][1], approx[i+2][0][0] - approx[i+1][0][0])

                
                               
                            elif index == len(approx) -1:
                                prev_angle = np.arctan2(approx[i][0][1] - approx[len(approx)-1 ][0][1], approx[i][0][0] - approx[len(approx) - 1][0][0])

                                stop_angle = np.arctan2(approx[i+1][0][1] - approx[i][0][1], approx[i+1][0][0] - approx[i][0][0])
                                next_angle = np.arctan2(approx[i+2][0][1] - approx[i+1][0][1], approx[i+2][0][0] - approx[i+1][0][0])

                                prev_diff = np.abs(prev_angle - stop_angle)
                                
                                next_diff = np.abs(prev_angle - next_angle)

                            continue
                            if np.abs(0- np.abs(stop_angle)) < 0.2 or np.abs( np.pi * 2 - np.abs(stop_angle)) < 0.2 or  np.abs( np.pi * 1 - np.abs(stop_angle)) < 0.2:
                                   
                                    
                                    if prev_diff > 1.4 or next_diff > 1.4:
                                        self.stop_line.append(approx[i-1:i+1])
                                        print(f"found {stop_angle} ")
                                        continue
                                        
                            print(f"found nothing {stop_angle}")
                            for i in range(1, len(approx)):
                                self.split_contours.append(approx[i-1:i+1])


                    if has_stop_line == True and False:
                        for i in range(1,len(approx)):
                            if i == index:
                                continue
                            length = cv2.norm(approx[i-1], approx[i])/ self.pixel_pro_meter
                            angle = np.arctan2(approx[i][0][1] - approx[i-1][0][1], approx[i][0][0] - approx[i-1][0][0])  
                            if length < 0.26:
                        #        print("length" + str(length))
                                continue

                            if np.abs(stop_angle - angle) < 1.0 or np.abs(stop_angle - angle) > 2.7:
                        #        print(" alg diffrence " + str(np.abs(stop_angle-angle )))
                                continue
                         #   print(np.abs(stop_angle - angle))    

                            self.split_contours.append(approx[i-1:i+1])
                    else:
                        print("not a line")
                        for i in range(1, len(approx)):
                            self.split_contours.append(approx[i-1:i+1])
                            # Draw the line and width on the original image
            else:
                cont_obj = self.all_lane_obj.pop(j)
                hull = cv2.convexHull(cont_obj.contour,clockwise=False)
                length = cv2.arcLength(hull,True)
                # Check if the length is greater than 4
                if length > 4:
                    # Process each part of the convex hull separately
                    for i in range(1, len(hull)):
                        # Calculate the length of the convex hull segment
                        segment_length = cv2.norm(hull[i-1], hull[i])

                        # Calculate the width of the convex hull segment
                        width = 0
                        for j in range(len(hull[i-1])):
                            point1 = hull[i-1][j]
                            point2 = hull[i][j]
                            width += cv2.norm(point1, point2)

                        # Normalize the width based on the number of points
                        width /= len(hull[i-1])

                        # Check if both length and width are greater than 4
                        if segment_length > 4 and width > 4:
                            # Save all points of the valid contour segment
                            self.split_contours.append(hull[i-1:i+1].reshape(-1, 2))

                            # Calculate the middle point of the contour segment
                            middle_point = np.mean(hull[i-1:i+1], axis=0).astype(int)
                            
                

    def check_if_is_halt_line(self,approx, i):

        length = cv2.norm(approx[i-1], approx[i])/ self.pixel_pro_meter
            
        if (len(approx) > 6) and length > 0.25 and length < 0.28:
            stop_angle = np.arctan2(approx[i][0][1] - approx[i-1][0][1], approx[i][0][0] - approx[i-1][0][0])
            
            # check if 
           
        else:
            return False



        


               # else:
                #    self.other_countours.append(self.all_lane_obj.pop(i))

    def is_side_lane_element(self,lane_contour):
        
        if lane_contour.lane_length < self.filter_middle_lines_obj.max_length * 2:
            lane_contour.debug_message_side_line += "contour length to small= " + str(lane_contour.lane_length) 
            return False
        if lane_contour.lane_length > 15:
            lane_contour.debug_message_side_line += "contour length to big = " + str(lane_contour.lane_length)
            return False

        if lane_contour.width > 1 and False:
            lane_contour.debug_message_side_line += "width = " + str(lane_contour.width)
            return False

        if lane_contour.line_fit_error >  170:
            lane_contour.debug_message_side_line += "line error = " + str(lane_contour.line_fit_error)
            return False
        return True








    def create_left_right_lanes(self):
        """
            Create a middle curve from a set of contours.

            This function processes a set of contours to create a middle curve. It sorts the contours
            by distance from a player point and filters them. It also separates contours into left and
            right lines and performs polynomial regression on them to create curves.

            Returns:
                None: The resulting curves are stored in attributes.
        """

        for x in self.middle_line_countours:
            for point in x.contour:

                self.middle_line_points.append(point[0])

        all_left = []
        all_right = []


        
        self.sorted_middle = [x.center[0] for x in self.middle_line_countours]
        
        
        self.sorted_middle.sort(key=lambda sorted_middle: sorted_middle[0])
        

        self.min_middle_x= self.width/2
        self.max_middle_x = self.width/2

        if len(self.sorted_middle) > 0:
            if (self.sorted_middle[0][0]< self.min_middle_x):
                self.min_middle_x = self.sorted_middle[0][0]

            if (self.sorted_middle[len(self.sorted_middle)-1][0]> self.max_middle_x):
                self.max_middle_x = self.sorted_middle[len(self.sorted_middle)-1][0]  
                
        self.middle_x_values= [x.center[0][0] for x in self.middle_line_countours] 
        self.middle_y_values=  [x.center[0][1] for x in self.middle_line_countours] 

        small_centroids_x = [contour.cx for contour in self.middle_line_countours]
        self.average_small_centroid_x = np.mean(small_centroids_x)

        for sor in self.possible_side_lines:
              
            if self.is_Left(sor,sor.center[0],sor.contour,sor.x_value_highest_row,sor.cx):
                self.left_line_countours.append(sor)
                all_left.append(sor.contour)
            else:         
                self.right_line_countours.append(sor)
                all_right.append(sor.contour)
        
        for sor in self.other_countours:
            continue
            if self.is_Left(sor,sor.center[0],sor.contour,sor.x_value_highest_row,sor.cx):
                self.left_line_countours.append(sor)
                all_left.append(sor.contour)
            else: 
                self.right_line_countours.append(sor)
                all_right.append(sor.contour)
        


    def get_longest_side_points(self,contour):
        # Calculate the perimeter (arc length) of the contour
        perimeter = cv2.arcLength(contour, True)

        # Approximate the contour with a polygon (reduce the number of points)
        epsilon = 0.02 * perimeter  # Adjust the epsilon value as needed
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Find the longest side
        longest_side = None
        max_length = 0


        has_break_line = False

        for i in range(len(approx)):
            p1 = approx[i][0]
            p2 = approx[(i + 1) % len(approx)][0]

            # Calculate the length of the current side
            length = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5

            if length > max_length:
                max_length = length
                longest_side = (p1, p2)

        # Extract all the points along the longest side
        points_on_longest_side = []

        if longest_side is not None:
            p1, p2 = longest_side
            dist = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
            step = 1.0 / dist

            for t in range(int(dist)):
                x = int(p1[0] + t * step * (p2[0] - p1[0]))
                y = int(p1[1] + t * step * (p2[1] - p1[1]))
                points_on_longest_side.append([x, y])

        return points_on_longest_side


    

    

    def sort_outer_lines(self):
        contours = [x.contour for x in self.right_line_countours]
        contours = sorted(contours, key=lambda x :cv2.minEnclosingCircle(x)[0][0]) 
        self.right_contour = contours[-1]




    def approximate_point_for_x(self,x_to_lookup):
        """
        Approximate a curve based on given x and y values and find the y value for a specific x.

        Args:
        x_values (list): List of x coordinates.
        y_values (list): List of corresponding y coordinates.
        x_to_lookup (float): The x value to look up.

        Returns:
        float: The approximated y value for the given x.
        """
        y_approximated = np.interp(x_to_lookup, self.middle_x_values, self.middle_y_values)
        return y_approximated




    @deprecated
    def is_left_of_curve(self, center):
        """
        Check if a point is left of a curve defined by x and y values.

        Args:
        point (tuple): The (x, y) coordinates of the point.
        Returns:
        bool: True if the point is left of the curve, False otherwise.
        """
        

        x = center[0]
        if x < self.min_middle_x or x + self.dx > self.max_middle_x:
                if x < self.min_middle_x:
                    seite = "left"
                    #return True
                else:
                    seite = "right"
                    #return False
            ##print(f"Punkt {seite}: {seite}-Seite der Kurve (außerhalb des Gültigkeitsbereichs).")
                return seite == "left"
        
    
        y_curve = self.approximate_point_for_x(center[0])
        #print(y_curve)
        if center[1] > y_curve:
            return True  # Der Punkt liegt links von der Kurve
        else:
            return False  # Der Punkt liegt rechts von der Kurve

    def schnittpunkt(self,p1, p2, p3):
        """
        Berechnet den Schnittpunkt einer linearen Funktion mit einer konstanten Funktion.

        Args:
            p1: Der erste Punkt der linearen Funktion.
            p2: Der zweite Punkt der linearen Funktion.
            p3: Der Punkt, an dem die konstante Funktion die x-Achse schneidet.

        Returns:
            Ein Tupel mit dem Schnittpunkt der beiden Funktionen.
        """
        # Berechnen der Steigung der linearen Funktion.
        m = (p2[1] - p1[1]) / ((p2[0] - p1[0]) + 0.0000001)
        # Berechnen des y-Wertes des Schnittpunktes.
        y = m * p3[0] + p1[1]
        # Berechnen des x-Wertes des Schnittpunktes.
        #x = (y - p1[1]) / m
        return y

    def average_x_of_contour(self,contour):
        """
        Calculate the average x-coordinate of a contour.

        Args:
            contour: A list of points representing the contour.

        Returns:
            float: The average x-coordinate of the contour points.
        """
        if len(contour) == 0:
            return 0.0
        elif len(contour) == 1:
            return contour[0][0]
        else:
            x_coordinates = [point[0] for point in contour]
            average_x = np.mean(x_coordinates)
        return float(average_x)




    def is_left_find_closed_elememt(self, cont_obj):

        
        check_if_non_bigger = False
        index = 0

        for middle_ele in self.middle_line_countours:
            if middle_ele.y_value_highest_row < cont_obj.y_value_highest_row:
                if index == 0:
                    # no element in the middle has a bigger 
                    if False:
                        if middle_ele.x_value_highest_row > cont_obj.x_value_highest_row:
                            return "left"
                        else:
                            return "right"
                break
            index = index + 1

        if index >= len(self.middle_line_countours):
            index = len(self.middle_line_countours) -1

       
        closed_obj = self.middle_line_countours[index]
        closed_obj_angle = closed_obj.deviation_angle
        
        
        
        angle_from_middle_to_obj = np.arctan2(closed_obj.x_value_highest_row - cont_obj.center[0][0],
                                    closed_obj.y_value_highest_row - cont_obj.center[0][1])

       # #print(f'angle_from_middle_to_obj = {angle_from_middle_to_obj} from middle {closed_obj.x_value_highest_row,cont_obj.y_value_highest_row} with angle{closed_obj_angle} to {cont_obj.x_value_highest_row,closed_obj.y_value_highest_row}')

        if angle_from_middle_to_obj > closed_obj_angle:
            return "left"
        else: 
            return "right"
        

    def predict_value(self,contour_object):
        x1, y1 = self.middle_line_countours[0].bottom_point
        x2, y2 = contour_object.bottom_point

        
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            

        angle = self.middle_line_countours[0].angle 

        if y1 < y2:        
            
            x_value = (distance * math.cos(angle))
            y_value = (distance * math.sin(angle))
            if y_value <= 0:
                x = x1  - x_value
                y = y1 -  y_value
                
                return x > contour_object.bottom_point[0]
            else:
                x = x1  + x_value
                y = y1 + y_value
                
               
                return x > contour_object.bottom_point[0]  
        else:
            
            x_value = (distance * math.cos(angle))
            y_value = (distance * math.sin(angle))
            if y_value >= 0:
                x = x1  - x_value
                y = y1 -  y_value
                
                return x > contour_object.bottom_point[0]
            else:
                x = x1  + x_value
                y = y1 + y_value
         
                return x > contour_object.bottom_point[0]  


       
    import numpy as np

    def vergleiche_contouren(self,contour1, contour2):
        # Reshape the contours to flatten them
        flat_contour1 = np.array(contour1).reshape(-1, 2)
        flat_contour2 = np.array(contour2).reshape(-1, 2)
        
        
                # Create dictionaries to map y-values to x-values for each contour
        contour1_dict = {point[1]: point[0]for point in flat_contour1}
        contour2_dict = {point[1]: point[0] for point in flat_contour2}

        
        # Iterate through y-values in contour1
        for y1, x1 in contour1_dict.items():
            # Check if the y-value exists in contour2
            if y1 in contour2_dict:
                x2 = contour2_dict[y1]

                # Compare x values
                if x1 < x2:
                    return "left"
                elif x1 > x2:
                    return "right"
                else:
                    return "right"
            else:
               return None





    def is_Left(self,sor_obj,center,cont, max_y_point,centroid1_x):

        """
        Check if a point is left of a curve defined by x and y values.

        Returns:
        bool: True if the point is left of the curve, False otherwise.
        """



        x = center[0]
       


        if len(self.middle_line_countours) == 0:
            horizontal_midpoint = self.width // 2  # Horizontaler Mittelpunkt des Bildes

            if sor_obj.cx < horizontal_midpoint:
               return True
            else:
                return False
        
        
        for i in range(len(self.middle_line_countours)):
            check_value = self.vergleiche_contouren(sor_obj.contour,self.middle_line_countours[i].contour)
            if check_value == "left":
                return  True
            elif check_value == "right":
                return False



        if max_y_point  < self.min_middle_x  or max_y_point > self.max_middle_x:
            if max_y_point < self.min_middle_x:
                seite = "left"
            else:
                seite = "right"
            return seite == "left"


       #print("hello " + str(x))
       

       #print("hello again " + str(x))
       # if center[1] > self.middle_line_countours[0].center[0][1]:
        return self.predict_value(sor_obj)
        
        
       
        #print("hello  !!!!!!")
        check_value = self.is_left_find_closed_elememt(sor_obj)
        if check_value == "left":
            return  True
        elif check_value == "right":
            return False
        else:
           
            if (False):
                erg = 0
                middle_cx = 0
                for con in self.middle_line_countours:
                    middle_cx += con.cx
                    if (centroid1_x<con.cx):
                        erg = erg + 1
                    else:
                        erg = erg - 1
                middle_cx = middle_cx/len(self.middle_line_countours)
            
                if erg >= 2:
                    return True
                elif erg * -1  >= 2:
                    return False   
            else:
                if (self.tangens_filter):    
                    schwellenwinkel = 60
                    self.dx = 0.5
                    kurven_y = self.approximate_point_for_x(center[0])
                    kurven_y_dx = self.approximate_point_for_x(center[0] + 0.5 )
                    tangente_winkel = np.arctan((kurven_y_dx - kurven_y) / self.dx)
                    punkt_winkel= np.arctan2(x - kurven_y, x)
                    winkel_differenz = np.abs(np.degrees((punkt_winkel - tangente_winkel)))
                    if winkel_differenz < schwellenwinkel:
                        return True
                    else:
                        return False
                elif(True):
                    average_x_of_middle_line = sum(self.middle_x_values) / len(self.middle_x_values)
                    average_center_x_of_of_middle_line = sum([x.center[0] for x in self.middle_line_countours]) / len(self.middle_line_countours)
                    ##print(f"con cx is {con.cx} for contour with center {center} max y point {max_y_point} and middle_line_avr is {average_center_x_of_of_middle_line}  middle line cx is {average_x_of_middle_line}")
                    
                    return max_y_point < average_x_of_middle_line
                


class LaneCountour:
    def __init__(self,
                  contour,
                  pixel_pro_meter,
                  height,
                  width):
        self.distance = 0
        self.contour = contour
        self.length = cv2.arcLength(self.contour, closed=True)
        self.lane_length = self.length / pixel_pro_meter
        self.contour_area = cv2.contourArea(self.contour) / pixel_pro_meter
        self.center = np.mean(contour, axis=0)
        
       
        M = cv2.moments(self.contour)
        if M["m00"] != 0:
            self.cx = int(M["m10"] / M["m00"])
            self.cy = int(M["m01"] / M["m00"])
        else:
             # the contour area is zero
            self.cx = self.cy = 0
            return

        

        self.center_world_x_cor = height -  self.cy * pixel_pro_meter
        self.center_world_y_cor = self.cx * pixel_pro_meter - width/2 
        
        self.debug_message_side_line = ""
        #M['mu11'] is a moment representing the product of the differences between
        #  the x and y coordinates of the contour points
        # Calculate the angle of the contour in radioan
        #M['mu20'] - M['mu02'] is another moment representing the second-order
        #  central moments of the contour.
        self.angle = 0.5 * np.arctan2(2 * M['mu11'], M['mu20'] - M['mu02'])

        # Berechnung von Aspect Ratio basierend auf Konturlänge und Fläche
        self.area_aspect_ratio = self.lane_length/ self.contour_area
        self.width = self.contour_area / self.lane_length


        # contour = Numpy array in shape (n,1,2)
        # n = number of points. 1 number of countour => mostly only one
        # reshape -1 => reshape first dimension
        # automaticly determente the size of dimension 
        # the 2 => because x and y 
        reshaped_contour = contour.reshape(-1,2)
        # standard abweichung
        self.x_std = np.std(reshaped_contour[:, 0])
        self.y_std = np.std(reshaped_contour[:, 1])

        # Calculate the angle between the x and y coordinates of the contour points.
        self.deviation_angle = np.arctan2(self.x_std, self.y_std)

        max_y_row = contour[:, :, 1].argmax()  # Get the row index with the maximum y-coordinate
        min_y_row = contour[:, :, 1].argmin()  # Get the row index with the maximum y-coordinate
        
        self.x_value_highest_row = contour[max_y_row][0][0] # x at the bottom of the picture
        self.y_value_highest_row = contour[max_y_row][0][1] # y at the bottom of the picutre

        self.bottom_point = [self.x_value_highest_row,self.y_value_highest_row]

        self.top_point = [contour[min_y_row][0][0],  contour[min_y_row][0][1]]


        contour_points = contour[:, 0, :]
        self.rows, cols = contour_points[:, 1], contour_points[:, 0] 
         

        line = np.polyfit(self.rows, cols, 1)
        self.line_fit_error = np.sum(np.abs(cols - np.polyval(line, self.rows))) / len(self.rows)

        