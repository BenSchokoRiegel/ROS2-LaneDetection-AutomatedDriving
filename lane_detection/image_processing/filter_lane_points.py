import time
import rclpy
import numpy as np

from geometry_msgs.msg import Pose, Quaternion, PoseArray, PoseStamped, Point

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Quaternion, PoseArray, PoseStamped, Point


from sklearn.linear_model import RANSACRegressor

import numpy as np
import cv2
from scipy.spatial import KDTree
from scipy.optimize import curve_fit
from scipy.interpolate import interp1d

from scipy.spatial import cKDTree

from lanelet_msgs.msg import Lanelet,LineString

import math


class Filter_Through_Lines:
    def __init__(self,
                 bird_eye_view,
                 img,
                 left_line_contours_obj, 
                 right_line_contours_obj,
                 middle_line_countours,
                 pixel_pro_meter,
                 player_point,
                 
                 max_distance_between_lane = 2.0,
                 num_of_point_per_meter = 50,
                 meter_to_predict = 3):
        self.lanelet = Lanelet()
        self.pixel_pro_meter = pixel_pro_meter
        self.max_distance = max_distance_between_lane 
        self.image = img
        self.player_point = player_point
        self.num_of_point_per_meter = num_of_point_per_meter
        self.meter_to_predict = meter_to_predict
       # print(self.image.shape)
        self.height, self.width = self.image.shape[0:2]

        self.bird_eye_view = bird_eye_view
         
        self.filter_all_points(left_line_contours_obj,right_line_contours_obj, middle_line_countours)
        return  
       #print(f'{len(left_line_contours_obj)} , {len(right_line_contours_obj)}')
        # First sort them by how far they are from the player points   
        for ele in left_line_contours_obj + right_line_contours_obj:
          
            distance = np.linalg.norm(np.array(player_point) - np.array(ele.bottom_point))
            ele.distance = distance
           #print(f'player_point{player_point}distance{distance}ele.bottom_point{ele.bottom_point}')

        left_line_contours_obj.sort(key=lambda left_line_contours_obj: left_line_contours_obj.distance)
        right_line_contours_obj.sort(key=lambda right_line_contours_obj: right_line_contours_obj.distance)
        
         
        self.left_line_contours_obj = self.filter_left_and_right_with_distance_between_obj(left_line_contours_obj)
        self.right_line_contours_obj = self.filter_left_and_right_with_distance_between_obj(right_line_contours_obj)

        
        #self.middle_covex_hole = cv2.convexHull(np.array([countour for countour in self.left_line_contours_obj.contour]))
       # self.left_covex_hole = cv2.convexHull(np.array([countour for countour in left_line_contours_obj.contour]))
       # self.right_covex_hole = cv2.convexHull(np.array([countour for countour in right_line_contours_obj.contour]))

        self.closest_points_left_to_right = []
        self.closest_points_right_to_left = []

            # Find closest points between contours using KDTree
        self.gen_middle_line = self.find_interpolate_those_points(
            self.left_line_contours_obj,  self.right_line_contours_obj
            #[x.contour for x in self.left_line_contours_obj], [x.contour for x in self.right_line_contours_obj], distance_threshold=15
        )
          
        #self.draw_points(self.image, [point for point, _ in self.gen_middle_line], color=(0, 255, 0))  # Green for closest_points_left_to_right
        #self.draw_points(self.image, [point for point, _ in self.closest_points_right_to_left], color=(255, 255, 0))  # Yellow for closest_points_right_to_left



    # Function to draw points on an image
    def draw_point_to_line(self,image, points, color=(0, 255, 0), radius=4):
        if points is None:
            return 
        #for point in points:
        #    cv2.circle(image, (int(point[0]),int(point[1])), radius, color, -1)
        for i in range(len(points) - 1):
                cv2.line(image, (int(points[i][0]), int(points[i][1])), (int(points[i + 1][0]), int(points[i + 1][1])), color, 2)

    def draw_points(self,image, points, color=(0, 255, 0), radius=4):
        if points is None:
            return 
        for point in points:
            cv2.circle(image, (int(point[0]),int(point[1])), radius, color, -1)

    # THis curve must end in the same angle as the first Element
    def curve_to_first_middle_line(self, start,contour_obj):
        
        start_x,start_y = start
        angle = contour_obj.deviation_angle
        #print(angle)
        # Startpunkt und Endpunkt
        
        end_x, end_y =  contour_obj.bottom_point
        

        # Vektor zwischen Startpunkt und Endpunkt
        delta_x = end_x - start_x
        delta_y = end_y - start_y

      #  print(f"delta_x {delta_x} delta_y {delta_y}")

        # Länge der Kurve
        curve_length = np.sqrt(delta_x**2 + delta_y**2)/self.pixel_pro_meter

        self.meter_to_predict -= curve_length
        num_points = curve_length*self.num_of_point_per_meter

        if num_points < 1 and num_points > 0:
            num_points = 1
        if num_points > 10:
            num_points = 10     

        #print(f"\n  \n {num_points}  \n \n")
        # Berechnung der Punkte entlang der Kurve
        t_values = np.linspace(0, 1, int(num_points))
        x_values = start_x + t_values * delta_x
        y_values = start_y + t_values * delta_y

        # Rotation der Kurve am Endpunkt
        x_end_rotated = end_x + curve_length * np.cos(angle)
        y_end_rotated = end_y + curve_length * np.sin(angle)

        # Lineare Interpolation zu den gedrehten Endpunkten
        x_values = np.linspace(start_x, x_end_rotated, int(num_points))
        y_values = np.linspace(start_y, y_end_rotated, int(num_points))

        # Punkte als Tupel speichern
        points_on_curve = np.array(list(zip(x_values.astype(dtype=int), y_values.astype(dtype=int))))

        return points_on_curve



    def calculate_start_point(self,contours_middle_start):
        x1, y1 = contours_middle_start.bottom_point
        x2, y2 = self.player_point
        y2 = y2 - 50 

        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        

        angle = contours_middle_start.angle 
       

        x_value = (distance * math.cos(angle))
        y_value = (distance * math.sin(angle))
        if y_value <= 0:
            x = x1  - x_value
            y = y1 -  y_value
        else:
            x = x1  + x_value
            y = y1 + y_value    
       
        return [x,y]




    # Firsts -> from the player_points the first contour
    def middle_curve_expand(self,contours_middle):

        
        if len(contours_middle) != 0:            
            points_middle =np.concatenate([contour.contour for contour in contours_middle])
            points_middle = np.squeeze(points_middle)
         
            points_middle = points_middle[::2] 
            points_to_next_contour = self.curve_to_first_middle_line(self.calculate_start_point(contours_middle[0]),contours_middle[0])
           #self.draw_points(self.image,points_to_next_contour,color=(255,255,255))
           # points_middle = np.a
            #for i in range(2,len(contours_middle)-1):



            return np.concatenate((points_middle, points_to_next_contour))
           
        else :
            points_middle = None
            return None

          
        



    def find_points_with_middle(self,kd_tree,points_middle):
        if len(points_middle) == 0:
            return 
        else:
            distance,nearest_indices_middle =  kd_tree.query(points_middle, k=1)
            return kd_tree.data[np.asarray(nearest_indices_middle.flatten(), dtype=int)] 


    def find_points_in_tree(self,kd_tree,points_middle):
        if points_middle is None:
            return []
        if len(points_middle) == 0:
            return []
        else:
            _, nearest_indices = kd_tree.query(points_middle, k=1)
            nearest_points = kd_tree.data[nearest_indices.flatten()]
        
            return nearest_points


    def draw_fps(self,elapsed_time, text,x):
        fps = 1/elapsed_time
        fps_text = f"FPS{fps:.2f}/{elapsed_time:.2} ms"
        cv2.putText(self.image, text + fps_text,(0,x),1,2,255,2)



    def filter_all_points(self,contours_left, contours_right, contours_middle):
        self.left_and_middle_points = None
        self.right_and_middle_points = None
        self.new_left_points = None
        self.new_right_points = None
        self.driver_lane = None
        self.middle_between_left_and_right = None
        self.driver_lane_right = None
        self.driver_lane_left = None
        self.middle_points_with_right = None

        self.new_right_points = None
        self.new_left_points = None

        start_time = time.time()
        points_middle = self.middle_curve_expand(contours_middle=contours_middle)
        
        kdtree_middle = None
        if len(contours_middle) != 0:
            kdtree_middle = cKDTree(points_middle) 
            
      #  print(f'middle shape {points_middle.shape}')
        #self.draw_points(self.image, points_middle,color=(0,0,255))
     #   print(points_middle)

        # Flatten the contours into a single array of points for KDTree
        # Flatten the contours into a single array of points
        if len(contours_left) != 0:
            points_left = np.vstack([np.squeeze(contour.contour, axis=1) for contour in contours_left])
            points_left = points_left[::10]
            kdtree_left = cKDTree(points_left)
            # maybe trop point drop point
            

            if kdtree_middle is not None:
                self.left_and_middle_points = self.find_points_in_tree(kdtree_left,points_middle=points_middle)
                self.middle_points_with_left =  self.find_points_in_tree(kdtree_middle,points_middle=self.left_and_middle_points)
                self.driver_lane_left = np.array([ [(r[0] +  ((r[0]-l[0])/2)), (r[1] +((r[1]-l[1])/2))] for r, l in zip(self.middle_points_with_left, self.left_and_middle_points)])


            #self.draw_points(self.image,points_left)
        else:
            kdtree_left = None


        if len(contours_right) != 0:
            points_right = np.vstack([np.squeeze(contour.contour, axis=1) for contour in contours_right])
            #points_right = points_right[::10]
            kdtree_right = cKDTree(points_right)
           
            if kdtree_middle is not None:
                self.right_and_middle_points = self.find_points_in_tree(kdtree_right,points_middle=points_middle)
                self.middle_points_with_right =  self.find_points_in_tree(kdtree_middle,points_middle=self.right_and_middle_points)
                self.driver_lane_right = np.array([ [(r[0]  +l[0]) / 2, (r[1] + l[1]) / 2] for r, l in zip(self.right_and_middle_points, self.middle_points_with_right)])

            #self.draw_points(self.image,points_right,color=(255,0,0))
        else:
            kdtree_right = None
          
        
        if kdtree_right is not None and kdtree_left is not None:
            self.new_right_points = self.find_points_in_tree(kdtree_right, points_left)
            self.new_left_points = self.find_points_in_tree(kdtree_left,self.new_right_points)
            

            self.middle_between_left_and_right = []
            self.driver_lane = []

            for r,l in zip(self.new_right_points,self.new_left_points ):
                self.middle_between_left_and_right.append([(r[0] +l[0]) / 2, (r[1] + l[1]) / 2])
                self.driver_lane.append([r[0] * 0.75 + l[0] * 0.25 , r[1] * 0.75 + l[1] * 0.25])
       
        elapsed_time = time.time() - start_time
        start_time = time.time()
      #  self.draw_fps(elapsed_time=elapsed_time, text=str("Filtering_points : "),x=int(self.image.shape[0] - 10))

        ## draw driver_lane_right
        self.draw_points(self.image, self.driver_lane_right ,color=(0,0,0))
      # #self.draw_points(self.image, self.right_and_middle_points ,color=(255,0,0))
       ##self.draw_points(self.image, self.middle_points_with_right ,color=(255,0,0))


        ## draw middle between left and right
        self.draw_points(self.image, self.driver_lane ,color=(0,0,0))
        #self.draw_points(self.image, self.new_left_points ,color=(0,255,0))
        #self.draw_points(self.image, self.new_right_points ,color=(0,255,0))

        self.draw_points(self.image, self.driver_lane_left ,color=(0,0,0))



        #self.draw_points(self.image, self.left_and_middle_points)
       ##self.draw_points(self.image, self.right_and_middle_points ,color=(255,0,0))
      # #self.draw_points(self.image,self.new_left_points,color=(0,255,0))
     #  #self.draw_points(self.image, self.new_right_points ,color=(255,0,0))
       # self.draw_point_to_line(self.image, self.driver_lane ,color=(255,255,255))
     
       ##self.draw_points(self.image, self.driver_lane_left ,color=(255,0,0))
       ##self.draw_points(self.image, self.middle_between_left_and_right ,color=(255,255,255))
        #self.draw_fps(elapsed_time=(time.time() - start_time), text=str("draw all Points : "),x=int(self.image.shape[0] - 50))

        #self.combine_with_RANSAC()
        self.create_lanelet()
        self.combine()



    def create_point(self,x,y,z=0):
        point = Point()

        x = (self.height - x) / self.pixel_pro_meter
        y = (self.width/2 - y) / self.pixel_pro_meter  
        point.x = float(x)
        point.y = float(y)
        point.z = float(0)
        return point

   # def draw_points_onto_image():



    
    def create_lanelet(self):
         
         
        #self.lanelet.right_lane.location_vector.append(self.create_point(point[0],point[1]))
       
       # middle and left  
        if self.driver_lane_left is not None:
            middle_and_left = np.array([  
                                        [
                                        self.create_point(m[0], m[1]),
                                        self.create_point(m[0] +  ((m[0]-l[0])),m[1] +((m[1]-l[1]))),
                                        self.create_point((m[0] +  ((m[0]-l[0])/2)), (m[1] +((m[1]-l[1])/2)))
                                        ]
                                        for m, l in zip(self.middle_points_with_left, self.left_and_middle_points)
                                        ])
        else:
            middle_and_left = np.empty((0, 3, 2))                                    
        
         # middle and right  
        if self.driver_lane_right is not None:
            middle_and_right = np.array([  
                                        [
                                        self.create_point(m[0], m[1]),
                                        self.create_point(r[0] +  ((r[0]-m[0])),r[1] +((r[1]-m[1]))),
                                        self.create_point((r[0] +  ((r[0]-m[0])/2)), (r[1] +((r[1]-m[1])/2)))
                                        ]
                                        for r, m in zip(self.right_and_middle_points, self.middle_points_with_right)
                                        ])
        else:
            middle_and_right = np.empty((0, 3, 2))    
                                                
        # right and left
        if self.driver_lane is not None:
            left_and_right = np.array([  
                                        [
                                        self.create_point((r[0] +l[0]) / 2, (r[1] + l[1]) / 2),
                                        self.create_point(r[0],r[1]),
                                        self.create_point(r[0] * 0.75 + l[0] * 0.25 , r[1] * 0.75 + l[1] * 0.25)
                                        ]
                                        for r, l in zip(self.new_right_points,self.new_left_points)
                                        ])
        else:
            middle_and_right = np.empty((0, 3, 2))


        self.right_middle_combined = np.concatenate([middle_and_left, middle_and_right, left_and_right], axis=0).tolist()

    
        self.right_middle_combined.sort(key=lambda x: (x[2].x**2 + x[2].y**2))

        transpose_array = np.transpose(np.array(self.right_middle_combined))
        
        self.lanelet.middle_lane.location_vector.extend(transpose_array[0])
        
        #print(self.lanelet)
        self.lanelet.right_lane.location_vector.extend(transpose_array[1])
        self.optimal_line_points = transpose_array[2]
        return
       # i = 0
        #for ele in right_middle_combined:
         #   print(f"{i } = {ele}")
          #  i = i + 1

     #   self.lanelet.right_lane.location_vector.append()
     #   self.lanelet.left_lane.location_vector.append()

        #print(np.array(right_middle_combined).shape)
       # print(right_middle_combined)
        
       






    




    def combine_with_RANSAC(self):

 # Combine all points from left, right, and middle
        all_points = []
        if self.driver_lane is not None:
            all_points.extend(self.driver_lane)

        if self.driver_lane_right is not None:
           all_points.extend(self.driver_lane_right)
        
        if self.driver_lane_left is not None:
            all_points.extend(self.driver_lane_left)

        if len(all_points) > 2:
            data = np.array(all_points)

            x,y = data[:,0],data[:,1]

            print(x)    
            ransac = RANSACRegressor()
            ransac.fit(data,y)

            if False:
                inlier_mask = ransac.inlier_mask_

                # get Points 
                inlier_points = data[inlier_mask]


                    # Zeichne alle Punkte in Grau
                for point in data:
                    break
                    print(point)
                    cv2.circle(self.image, point, 5, (169, 169, 169), -1)

                # Zeichne Inliers in Rot
                for point in inlier_points:
                    break
                    cv2.circle(self.image, point , 5, (0, 0, 255), -1)

            # Zeichne die RANSAC-Kurve in Blau
            curve_points = np.column_stack((x, ransac.predict(data)))
            curve_points = curve_points.astype(int)
            # Zeichne alle Punkte in Grau
           

            # Zeichne die RANSAC-Kurve in Blau
            for i in range(len(curve_points) - 1):
                cv2.circle(self.image, curve_points[i] , 5, (0, 0, 255), -1)
                #cv2.line(self.image, tuple(curve_points[i]), tuple(curve_points[i + 1]), (255, 0, 0), 2)



    def combine(self):
        # Combine all points from left, right, and middle
        all_x = []
        all_y = []

        #all_x.extend([self.player_point[0] for x in range(50,100)])
        #all_y.extend([self.player_point[1] + 200 - x for x in range(50,100)])

        if self.driver_lane is not None :
            all_x.extend(list(zip(*self.driver_lane))[0])
            all_y.extend(list(zip(*self.driver_lane))[1])

        if self.driver_lane_right is not None:
            all_x.extend(list(zip(*self.driver_lane_right))[0])
            all_y.extend(list(zip(*self.driver_lane_right))[1])

        if self.driver_lane_left is not None :
            all_x.extend(list(zip(*self.driver_lane_left))[0])
            all_y.extend(list(zip(*self.driver_lane_left))[1])


        try:    
            # Convert to numpy arrays
            all_x = np.array(all_x)
            all_y = np.array(all_y)


            # Use polynomial fitting with outlier weighting to approximate the curve
            degree = 2.5  # You can adjust the degree based on your data
            weights = np.ones_like(all_y)  # Start with equal weights for all points

            # Apply higher weights to points closer to the fitted curve
            fitted_curve = np.polyval(np.polyfit(all_x, all_y, degree), all_x)
            residual_errors = all_y - fitted_curve
            median_absolute_deviation = np.median(np.abs(residual_errors))
            outlier_indices = np.abs(residual_errors) > 3.0 * median_absolute_deviation
            weights[outlier_indices] = 0.1  # Lower weight for outliers

            # Set up the start condition
            #start_condition = np.polyfit([self.player_point[0] + 200], [self.player_point[1]], degree)

            # Combine the weights for fitting
            combined_weights = weights * (1 - np.exp(-0.5 * (all_x - self.player_point[0]) ** 2 / 100))

            # Use polynomial fitting with combined weights
            coefficients = np.polyfit(all_x, all_y, degree, w=combined_weights)

            # Generate points along the fitted curve
            fitted_y = np.linspace(min(all_y), max(all_y), 1000)
            fitted_x = np.polyval(np.polyfit(all_y, all_x, degree), fitted_y)

            # Draw the fitted curve on the image
            for i in range(len(fitted_x) - 1):
                cv2.line(self.image, (int(fitted_x[i]), int(fitted_y[i])), (int(fitted_x[i + 1]), int(fitted_y[i + 1])), (255, 0, 0), 3)

            
        except Exception as e:
            # Code to handle other types of exceptions
            #print(f"An error occurred: {e}")
            x = 333




    def find_closest_points_between_contours(self,contours_left, contours_right, distance_threshold=1000):
       #print(len(contours_left))
        
       
        # Flatten the contours into a single array of points for KDTree
        # Flatten the contours into a single array of points
        flat_contours_left = np.vstack([np.squeeze(contour.contour, axis=1) for contour in contours_left])
        flat_contours_right = np.vstack([np.squeeze(contour.contour, axis=1) for contour in contours_right])

        # Print the shapes
        
        # Build KDTree for flat_contours_right
        kdtree_right = cKDTree(flat_contours_right)
        kdtree_left = cKDTree(flat_contours_left)

        closest_points_left_to_right = []
        closest_points_right_to_left = []

        indexesnearest_points_in_left, distances_set2 = kdtree_left.query(flat_contours_right, k=1)
        
        nearest_points_in_right, distances_set2 = kdtree_right.query(flat_contours_left, k=1)



        return [],[]

        # Query KDTree for each point in flat_contours_right
        for point_right in flat_contours_left:
            # Query the KDTree for the closest point in flat_contours_left
            _, close_index_left = kdtree_left.query(point_right,k=1)

            # Get the closest point in flat_contours_left
            close_point_left = flat_contours_left[close_index_left]
            
            # Check if the distance is below the threshold
            distance = np.linalg.norm(point_right - close_point_left)
            if distance < distance_threshold:
                closest_points_right_to_left.append((point_right, close_point_left))

        return closest_points_left_to_right, closest_points_right_to_left

            









    def connect_closest_points(list_of_contours_left, list_of_contours_right):
        return 
        # Extrahieren Sie die Punkte aus den Konturen
        points1 = np.squeeze(list_of_contours_left)
        points2 = np.squeeze(list_of_contours_right)



        # Verwenden Sie den Nearest-Neighbor-Algorithmus, um für jeden Punkt in einer Kontur den am nächsten gelegenen Punkt in der anderen Kontur zu finden
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(points2)
        distances, indices = nbrs.kneighbors(points1)



        # Finden Sie die Indizes der Punkte, die am nächsten beieinander liegen
        min_distance_index = np.argmin(distances)

        # Extrahieren Sie die beiden am nächsten gelegenen Punkte
        closest_point_contour1 = points1[min_distance_index]
        closest_point_contour2 = points2[indices[min_distance_index][0]]

        return closest_point_contour1, closest_point_contour2



    

    # check if distance for points 
    def filter_left_and_right_with_distance_between_obj(self,contour_obj_list):
         
        last_point_top = None 
        for i in range(len(contour_obj_list)):
            if i == 0 :
                last_point_top = contour_obj_list[i].top_point
                continue
            
            ele = contour_obj_list[i]
            distance = np.linalg.norm(np.array(last_point_top) - np.array(ele.bottom_point))
            
            
            if distance > self.max_distance * self.pixel_pro_meter:
                contour_obj_list.pop(i)
                continue

            last_point_top = ele.top_point

        return contour_obj_list


        ### create the Line between the middle line contours 
        def complete_middle_line():
            pass





    def approximate_contour_with_fixed_points(contour, target_points):

        min_epsilon = 0
        max_epsilon = 1
        epsilon = (min_epsilon + max_epsilon) / 2

        while True:
            approx_curve = cv2.approxPolyDP(contour, epsilon, True)
            num_points = len(approx_curve)

            if num_points == target_points or max_epsilon - min_epsilon < 0.0001:
                break
            elif num_points < target_points:
                max_epsilon = epsilon
            else:
                min_epsilon = epsilon

            epsilon = (min_epsilon + max_epsilon) / 2

        return approx_curve


    def get_approximation_for_line_of_approx(contour):
        """
        Approximate a set of contours with a polygon.

        This function takes a list of contours, combines them into a single numpy array,
        and approximates the combined contour with a polygon.

        Args:
            all_contours: A list of contours to be approximated.

        Returns:
            ndarray: The approximated curve represented as a polygon.
        """
        # Convert the list of contours to a single numpy array
        contour = np.concatenate(contour)

        # Approximate the combined contour with a polygon
        #epsilon = 3  # Adjust this value for the desired level of approximation
        epsilon = 0.02 * cv2.arcLength(contour,True)
        approximated_curve = cv2.approxPolyDP(contour, epsilon, True)

        # Draw the approximated curve on the image
        return approximated_curve