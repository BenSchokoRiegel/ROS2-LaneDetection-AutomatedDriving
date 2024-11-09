import math
import time

import cv2
import rclpy

from skspatial.objects import Plane, Line
import tf2_ros
import numpy as np
import tf2_geometry_msgs.tf2_geometry_msgs
from geometry_msgs.msg import Twist, PointStamped,Point
from image_geometry.cameramodels import PinholeCameraModel


class BaseLinkTransformation:
    def __init__(
            self,
            msg,
            transform_stamped,
            matrix_points_x = 100,
            matrix_points_y = 100,
            max_allowed_distance_forward = 5,
            max_allowed_distance_sideways = 0.9,
            pixel_per_meter = 600.0
            ) -> None:
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)
        self.R = np.array(msg.r).reshape((3, 3))
        self.P = np.array(msg.p).reshape((3,4))
        
        self.allowed_distance_forward = max_allowed_distance_forward
        self.allowed_distance_sideways = max_allowed_distance_sideways
        self.H = None # Homogene Matrix
        self.orginal_points_on_x = matrix_points_x
        self.orginal_points_on_y = matrix_points_y
    
        self.pinhole_camera_model = PinholeCameraModel()
        self.pinhole_camera_model.fromCameraInfo(msg)
        self.transform_stamped = transform_stamped

        self.pixel_per_meter = pixel_per_meter

        # camera origin Point   
        ps_origin = PointStamped()
        p_origin = Point(x=0.0,y=0.0,z=0.0)
        ps_origin.point = p_origin
    
        self.transform_camera_origin = tf2_geometry_msgs.do_transform_point(ps_origin, self.transform_stamped)
        self.plane = Plane(point=[0, 0, 0], normal=[0, 0, 1])
        self.create_homography_matrix()



    def get_worldcordinate_for_pixel(self,point):
         # Berechnung der Richtung des Strahls der durch die Pixel X,Y geht
        ray = self.pinhole_camera_model.projectPixelTo3dRay(point)
        p = PointStamped()
        point = Point(x = ray[0],y = ray[1],z = ray[2])
        p.point = point 
        # Wo ist der Punkt im Raum 
        transformed_point = tf2_geometry_msgs.do_transform_point(p, self.transform_stamped)

        line_from_camera_to_bottom_for_x_y = Line.from_points(
            [transformed_point.point.x,transformed_point.point.y,transformed_point.point.z], 
            [self.transform_camera_origin.point.x,self.transform_camera_origin.point.y,self.transform_camera_origin.point.z])
        # Gibt wieder wo der Punkt auf eine Flache Ebene Trifft
        return self.plane.intersect_line(line_from_camera_to_bottom_for_x_y)
            

    


    @DeprecationWarning
    def image_point_to_camera_coordinates(self,string,point):
        
        try:
            # Annahmen: Kamera ist im Frame "camera_frame"
            # Warten Sie auf die Transformationsinformationen
            self.tf_buffer.can_transform(source_frame="camera_link", target_frame="base_link", time=rclpy.duration.Duration(),timeout=rclpy.duration.Duration(seconds=1))

            # Erhalten Sie die Transformation von der Kamera zum Weltkoordinatensystem base_link
            transform_stamped = self.tf_buffer.lookup_transform(source_frame="camera_link", target_frame="base_link", time=rclpy.duration.Duration(),timeout=rclpy.duration.Duration(seconds=1))


            # Annahmen: pinhole_camera ist Ihre PinholeCameraModel-Instanz mit Kameraparametern
            ray = self.pinhole_camera_model.projectPixelTo3dRay(point)
            p = PointStamped()
            point = Point(x = ray[0],y = ray[1],z = ray[2])
            #p.header.frame_id = self.call_back_information['frame_id']  
            #p.header.stamp = self.get_clock().now().to_msg()
 
            p.point = point
            
            # Transformieren Sie die 3D-Koordinaten vom Kamerakoordinatensystem ins Weltkoordinatensystem
            transformed_point = tf2_geometry_msgs.do_transform_point(p, transform_stamped)

            ps_origin = PointStamped()
            p_origin = Point(x=0.0,y=0.0,z=0.0)
            ps_origin.point = p_origin
           # ps_origin.header.frame_id = self.call_back_information['frame_id']  
            #ps_origin.header.stamp = self.get_clock().now().to_msg()
            transform_camera_origin = tf2_geometry_msgs.do_transform_point(ps_origin, transform_stamped)
            
            line_from_camera_to_bottom_for_x_y = Line.from_points(
                [transformed_point.point.x,transformed_point.point.y,transformed_point.point.z], 
                [transform_camera_origin.point.x,transform_camera_origin.point.y,transform_camera_origin.point.z])
            
           # print("line_from_camera_to_bottom_for_x_y" +  str(line_from_camera_to_bottom_for_x_y))

            plane = Plane.from_points([0,0], [1,0],[0,1])
            point_intersect = plane.intersect_line(line_from_camera_to_bottom_for_x_y)

            return(string + str(point_intersect))
        
        except tf2_ros.LookupException as e:
            print("Fehler beim Lookup der Transformation: " + str(e))
            return None, None, None
        except tf2_ros.ConnectivityException as e:
            print("Fehler beim Aufbau der Transformationsverbindung: " + str(e))
            return None, None, None
        except tf2_ros.ExtrapolationException as e:
            print("Fehler bei der Extrapolation der Transformation: " + str(e))
            return None, None, None



    def projectPixelTo3dRay(self, point_u_v):
        return self.pinhole_camera_model.projectPixelTo3dRay(point_u_v)


    def create_pinhole_model():
        PinholeCameraModel()
        
    def draw_on_picture_points(self,cv_image):
        for x in range(0,self.pinhole_camera_model.width,self.pinhole_camera_model.width//self.orginal_points_on_x):
            for y in range(0,self.pinhole_camera_model.height,self.pinhole_camera_model.height//self.orginal_points_on_y):
                point = [x, y]
                calc_cor = self.get_worldcordinate_for_pixel(point=point)
                if (calc_cor[0] < 0):
                    color = (0,0,0)
                else:
                    color=(255,0,0)

                cv2.circle(cv_image, center=point, radius=10, color=color, thickness=-1)  # radius-10 für einen vollen Kreis
                text = "x={:.1f}, y={:.1f}, z={:.1f}".format(calc_cor[0], calc_cor[1], calc_cor[2])
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.333
                font_thickness = 1
                text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
                text_x = point[0] + 15  # Shift the text to the right of the circle
                text_y = point[1] + text_size[1] // 2  # Center the text vertically
                cv2.putText(cv_image, text, (text_x, text_y), font, font_scale, color, font_thickness)

        return cv_image    

    @DeprecationWarning
    def create_matrix_for_bird_eye_view(self, original_points,transformed_points):
        
        bird_eye_transformed_points = []
        bird_eye_points = []
        # World Coordinate middle Down does not change 
      #  bird_eye_points += [self.pinhole_camera_model.width / 2, self.pinhole_camera_model.height]
       # bird_eye_transformed_points += [self.pinhole_camera_model.width / 2, self.pinhole_camera_model.height]

        # Middle Top over self.max height
       # bird_eye_points += [self.pinhole_camera_model.width / 2, self.max_x_height]
       # bird_eye_transformed_points += [self.pinhole_camera_model.width/2, 0]


        for original_point, transformed_point in zip(original_points, transformed_points):
            bird_eye_points += [original_point]
            transformed_y_value = self.pinhole_camera_model.height - (self.pinhole_camera_model.height * transformed_point[0] / self.allowed_distance_forward)
            if transformed_point[0] < 0:  
                transformed_x_value = self.pinhole_camera_model.width/2 + (self.pinhole_camera_model.width/2 * abs(transformed_point[1]) / abs(self.min_y))   
            else:
                 transformed_x_value = self.pinhole_camera_model.width/2 - (self.pinhole_camera_model.width/2 * transformed_point[1] / self.max_y)
            bird_eye_transformed_points += [[transformed_x_value,transformed_y_value]]
            #print(f'normal = {original_point:} with {transformed_point:} is tranformed_point \n = x = {transformed_x_value} y= {transformed_y_value}')
            

        from_ = np.array([bird_eye_points], dtype=np.float32)
        to = np.array([bird_eye_transformed_points], dtype=np.float32)    
        self.H , _ = cv2.findHomography(from_, to, cv2.RANSAC, 5.0)
            




    def create_homography_matrix(self):
        """
        Create a homography matrix H for the eagle view

        This function computes the homography matrix H based on specified transformation points
        and the input image's height.

        Args:
            image: The input image.
            image_height: The height of the input image.

        Returns:
            None: The homography matrix H is stored in the object's attribute.
        """
        start_time = time.time()        
        if self.pinhole_camera_model.resolution is not None:
            #original_points = np.array([[self.far_left, image_height], [self.far_right, image_height], [self.close_left, 0], [self.close_right, 0]], dtype=np.float32)
            #transformed_points = np.array([[self.close_left, image_height], [self.close_right, image_height], [self.close_left, 0], [self.close_right, 0]], dtype=np.float32)
            original_points = []
            transformed_points = []
            eagle_transformed = []
           
            self.max_x_height = 0.0
            self.max_x = 0.0
            self.min_x = 1000
            self.min_y = -0.01
            self.max_y = 0.0  

            for x in range(0,self.pinhole_camera_model.width,self.pinhole_camera_model.width//self.orginal_points_on_x):
                for y in range(0,self.pinhole_camera_model.height + 1,self.pinhole_camera_model.height//self.orginal_points_on_y):
                
                    
                   
                    point = [x, y]
                    transformed_point = self.get_worldcordinate_for_pixel(point=point)

                    if  transformed_point[0] >= 0 and transformed_point[0] <= self.allowed_distance_forward and abs(transformed_point[1]) <= self.allowed_distance_sideways:
                        
                        if transformed_point[0] > self.max_x:
                            self.max_x = transformed_point[0]
                            self.max_x_height = y
                        if transformed_point[1] < self.min_y:
                            self.min_y = transformed_point[1]
                        if transformed_point[0] < self.min_x:
                            self.min_x = transformed_point[0]    
                        if transformed_point[1] > self.max_y:
                            self.max_y = transformed_point[1]

                        eagle_transformed_y_value = self.pinhole_camera_model.height - (self.pixel_per_meter * transformed_point[0])
                      #  print(self.pixel_per_meter * transformed_point[0])
                        eagle_transformed_x_value = self.pinhole_camera_model.width/2 - (self.pixel_per_meter * transformed_point[1])
                     #   print(eagle_transformed_x_value)   
                          

                        original_points += [point]  
                        transformed_points +=  [[x for x in transformed_point[:2]]]
                        eagle_transformed += [[eagle_transformed_x_value,eagle_transformed_y_value]]
                     #   print(f'point = {point} is tranformed_point = {transformed_point[:2]}')
                  
           # self.pixel_per_meter = self.pinhole_camera_model.height/ self.max_x

            print(f'self.max_x {self.max_x} minx {self.min_x} min_y {self.min_y} max x{self.max_y} {self.max_x_height}')        

            self.original_points = np.array([original_points], dtype=np.float32)
            self.transformed_points = np.array([transformed_points], dtype=np.float32)
            self.eagle_transformed = np.array([eagle_transformed], dtype=np.float32)
            # dank diesem ransacReprojThreshold was für für den Reprojektionsfehlerschwellenwert steht können wir dann die rechenzeit reduzieren und extrem genau sein
            # vorher  nehmen wir noch 2 weit entfernte Punkte und geben uns damit die maßeinheit vom Bild 
            #dann können wir direkt aus dem birdview bild die 3D koordinaten lesen ohne es nochmal umformen zu müssen
            self.H , _ = cv2.findHomography(self.original_points,self.eagle_transformed, cv2.RANSAC, 5.0)
            self.H_for_pixel_in_points , _ = cv2.findHomography(self.original_points, self.transformed_points, cv2.RANSAC, 5.0)
            elapsed_time = time.time() - start_time
            print(f"Function execution took {elapsed_time} seconds")
           
           
    def cut_eagle_view_borders(self,image):  
        
        # Erstellen Sie eine leere Maske mit den gleichen Dimensionen wie das Bild
        mask = np.ones_like(image)

        # Setzen Sie den Schwellenwert für x- und y-Koordinaten
        #print(self.max_x)
        x_threshold_min, x_threshold_max =  int((image.shape[1]/2) - (self.max_y* self.pixel_per_meter)), int(image.shape[1]/2 + (abs(self.min_y) * self.pixel_per_meter))
        y_threshold_max =  image.shape[0]- int(self.max_x * self.pixel_per_meter)

      #  print({x_threshold_min}, {x_threshold_max},{y_threshold_max})
      
        #mask[y_threshold_min:y_threshold_max, x_threshold_min:x_threshold_max, :] = image[y_threshold_min:y_threshold_max, x_threshold_min:x_threshold_max, :]

        
        if x_threshold_min > 0:
            mask[:, :int(x_threshold_min), :] = 0  # Left Side
        if x_threshold_max < image.shape[1]:
            mask[:, int(x_threshold_max):, :] = 0  # Right Side 
        if y_threshold_max < image.shape[0]:
            mask[0:int(y_threshold_max), :, :] = 0  # 
        
        masked_image = image * mask

        return masked_image


    def undistort_image(self, image):
        """
        Undistort an input image using camera parameters.
        """
        undistorted_image = cv2.undistort(image, self.K, self.D)
        return undistorted_image


    def reverse_transform_image(self, transformed_image):
        """
        reverse the eagle view for showing
        """

        image_width = transformed_image.shape[1]
        image_height = transformed_image.shape[0]

        # Reverse the transformation using the inverse homography matrix
        reverse_transformed_image = cv2.warpPerspective(transformed_image, np.linalg.inv(self.H), (image_width, image_height), flags=cv2.INTER_LINEAR)
        return reverse_transformed_image


    def transform_image(self, raw_image):
        """
        Apply a perspective transformation to an input image to create the eagle view

        Args:
            raw_image: The original input image.

        Returns:
            ndarray: The transformed image.
        """
        image_width = raw_image.shape[1]
        image_height = raw_image.shape[0]
        
        

        # Apply the transformation to the original image
        transformed_image = cv2.warpPerspective(raw_image, self.H, (image_width, image_height), flags=cv2.INTER_LINEAR)
        return self.cut_eagle_view_borders(transformed_image)


    # #TODO later
    def transform_per_pov(self,width,height):
        znear = 0.05
        f = (width / 2) / np.tan(self.fov / 2)

        # Step 2: Create the homography matrix H
        H = np.array([
            [f, 0, width / 2],
            [0, f, height / 2],
            [0, 0, 1]
        ])     



        """ point = (1440/2,1080/2)
        string = f"pixel (x={point[0]} y={point[0]}) => "

        

        point = (1440/1.3,1080/2)
        string = p(x=x y=y)
        self.get_logger().warn(self.eagle_view.image_point_to_camera_coordinates("mitte ",point))
        
        point = (1440/1.3,1080/3.0)
       # self.get_logger().warn(self.eagle_view.image_point_to_camera_coordinates("mitte ",point))

        point = (1440/1.3,1080/1.0)
        #self.get_logger().warn(self.eagle_view.image_point_to_camera_coordinates("mitte ",point))

    """



    """
    def get_intersection_point(self,print_out,point, camera_height=1.0,camera_angle=1.74379):
        self.get_logger().log(str(print_out),  LoggingSeverity.INFO)
        # Berechnung der Richtung des Strahls der durch die Pixel X,Y geht
        ray_direction = self.eagle_view.pinhole_camera_model.projectPixelTo3dRay(point)
        self.get_logger().log("ray_dir for " + str(point) + " = " + str(ray_direction), LoggingSeverity.INFO)
        # Berechnung des Skalars parameters t 
        # wird verwendet um um Schnittpunkt eines Strahls mit Ebene zu berechnen
        # gibt an wie lang der Strahl ist
        t = camera_height / math.tan(camera_angle)
        self.get_logger().log("t = " + str(t) , LoggingSeverity.INFO)
        intersection_point = (ray_direction[0] * t, ray_direction[1] * t)
        self.get_logger().log("intersection_point= " + str(intersection_point) , LoggingSeverity.INFO)
    """
