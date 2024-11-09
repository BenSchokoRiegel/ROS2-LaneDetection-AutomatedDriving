import rclpy
import numpy as np

from geometry_msgs.msg import Pose, Quaternion, PoseArray, PoseStamped, Point

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Quaternion, PoseArray, PoseStamped, Point

import numpy as np
import cv2

from sklearn.neighbors import NearestNeighbors


class Filter_Through_Lines_2:
    def __init__(self,
                 left_line_contours, 
                 right_line_contours,
                 middle_line_countours,
                 pixel_pro_meter):
        self.pixel_pro_meter = pixel_pro_meter
        
        
        left_line_contours.sort(key=lambda left_line_contours: left_line_contours.y_value_highest_row)
        right_line_contours.sort(key=lambda right_line_contours: right_line_contours.y_value_highest_row)



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

    def use_approximation_to_create_lane(all_contours):
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
        all_contours = np.concatenate(all_contours)

        # Approximate the combined contour with a polygon
        #epsilon = 3  # Adjust this value for the desired level of approximation
        epsilon = 0.02 * cv2.arcLength(all_contours,True)
        approximated_curve = cv2.approxPolyDP(all_contours, epsilon, True)

        # Draw the approximated curve on the image
        return approximated_curve


        
    def connect_closest_points(contour1, contour2):
        # Extrahieren Sie die Punkte aus den Konturen
        points1 = np.squeeze(contour1)
        points2 = np.squeeze(contour2)

        # Verwenden Sie den Nearest-Neighbor-Algorithmus, um für jeden Punkt in einer Kontur den am nächsten gelegenen Punkt in der anderen Kontur zu finden
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(points2)
        distances, indices = nbrs.kneighbors(points1)

        # Finden Sie die Indizes der Punkte, die am nächsten beieinander liegen
        min_distance_index = np.argmin(distances)

        # Extrahieren Sie die beiden am nächsten gelegenen Punkte
        closest_point_contour1 = points1[min_distance_index]
        closest_point_contour2 = points2[indices[min_distance_index][0]]

        return closest_point_contour1, closest_point_contour2





















# last minute function. saddly this has not a dokumenation jet
class PrepareForPublisher:
    def __init__(self,
                 lanelet, 
                 left_line_contours, 
                 right_line_contours,
                 middle_line_points, 
                 points_per_lane):
        
        self.left_line_contours = left_line_contours
        self.right_line_contours = right_line_contours
        self.left_line_points = []
        self.right_line_points = []
        self.middle_line_points = []
        
        self.data = lanelet
        if len(self.left_line_contours) > 0:
            self.left_line_points = self.find_max_x_for_y([self.left_line_contours[0]])
        if len(self.right_line_contours) > 0:
             self.right_line_points = self.find_min_x_for_y([self.right_line_contours[0]])

        if (len(self.right_line_points) > 0):
            if  points_per_lane >= len(self.right_line_points):
                self.right_line_points = self.right_line_points
            else:
                step = len(self.right_line_points) // points_per_lane  
                self.right_line_points = self.right_line_points[::step]  
            
          
            if (len(self.right_line_points) > 0):
                for point in self.right_line_points:
                    self.data.right_lane.location_vector.append(
                        self.create_point(point[0],point[1]))
        
        elif (len(self.left_line_points) > 0):
            if   points_per_lane >= len(self.left_line_points):
                self.left_line_points = self.left_line_points
            else:
                step = len(self.left_line_points) // points_per_lane  
                self.left_line_points = self.left_line_points[::step]    
        
            if (len(self.left_line_points) > 0):
                for point in self.left_line_points:
                    self.data.right_lane.location_vector.append(
                        self.create_point(point[0],point[1]))
        
        if   points_per_lane >= len(middle_line_points):
            self.middle_line_points = middle_line_points
        else:
            step = len(middle_line_points) // points_per_lane  
            self.middle_line_points = middle_line_points[::step]  
        
        if len(self.middle_line_points)  > 0:
                for point in self.middle_line_points:
                    self.data.middle_lane.location_vector.append(
                        self.create_point(point[0],point[1])) 
        



    def create_point(self,x,y,z=0):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(0)
        return point
   


    def find_min_x_for_y(self,contours):
        # Erstelle eine leere Liste für die Ergebnisse
        min_x_for_y = []

        # Iteriere durch die Konturen
        for c in contours:
            for point in c.contour:
                x, y = point[0]  # Extrahiere x- und y-Koordinaten des Punktes

                # Füge das x und y als Punkt zur Liste hinzu
                min_x_for_y.append([x, y])

        # Sortiere die Liste nach der y-Koordinate
        min_x_for_y.sort(key=lambda point: point[1])

        # Gruppiere die Punkte nach der y-Koordinate und speichere das Minimum für jedes y
        min_x_dict = {}
        for point in min_x_for_y:
            x, y = point
            if y in min_x_dict:
                min_x_dict[y] = min(min_x_dict[y], x)
            else:
                if (y  % 2 == 0):
                    min_x_dict[y] = x

        # Erstelle die finale Liste von Punkten [x, y]
        result = [[min_x, y] for y, min_x in min_x_dict.items()]

        return result

    def find_max_x_for_y(self,contours):
        # Erstelle eine leere Liste für die Ergebnisse
        max_x_for_y = []

        # Iteriere durch die Konturen
        for c in contours:
            for point in c.contour:
                x, y = point[0]  # Extrahiere x- und y-Koordinaten des Punktes

                # Füge das x und y als Punkt zur Liste hinzu
                max_x_for_y.append([x, y])

        # Sortiere die Liste nach der y-Koordinate
        max_x_for_y.sort(key=lambda point: point[1])

        # Gruppiere die Punkte nach der y-Koordinate und speichere das Maximum für jedes y
        max_x_dict = {}
        for point in max_x_for_y:
            x, y = point
            if y in max_x_dict:
                max_x_dict[y] = max(max_x_dict[y], x)
            else:
                if (y  % 2 == 0):
                    max_x_dict[y] = x

        # Erstelle die finale Liste von Punkten [x, y]
        result = [[max_x, y] for y, max_x in max_x_dict.items()]
        
        return result

    import numpy as np

    def perform_polynomial_regression(self,points, degree=2):

        if not points:
            return points
        if any(len(point) != 2 for point in points):
            return points
        
        x_values, y_values = zip(*points)
        coefficients = np.polyfit(x_values, y_values, degree)
        poly_function = np.poly1d(coefficients)

        x_fit = np.linspace(min(x_values), max(x_values), 100)
        y_fit = poly_function(x_fit)

        fitted_points = list(zip(x_fit, y_fit))

        return fitted_points

        

    def use_approximation_to_create_lane(all_contours):
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
        all_contours = np.concatenate(all_contours)

        # Approximate the combined contour with a polygon
        epsilon = 3  # Adjust this value for the desired level of approximation
        approximated_curve = cv2.approxPolyDP(all_contours, epsilon, True)

        # Draw the approximated curve on the image
        return approximated_curve
    



    def approximate_contours_to_line(self,contours):
        # Kombiniere alle Konturen zu einer einzigen Liste von Punkten
        all_points = np.concatenate(contours)

        # FitLine, um die Linie zu approximieren
        [vx, vy, x, y] = cv2.fitLine(all_points, cv2.DIST_L2, 0, 0.01, 0.01)

        # Berechne die Punkte auf der approximierten Linie
        line_points = []
        for i in range(len(all_points)):
            point_on_line = (x + i * vx, y + i * vy)
            line_points.append(point_on_line)

        return line_points

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = (
        np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
        - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    )
    qy = (
        np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    )
    qz = (
        np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    )
    qw = (
        np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
        + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    )
    return [qx, qy, qz, qw]





