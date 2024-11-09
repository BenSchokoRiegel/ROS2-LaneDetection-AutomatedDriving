import argparse
import traceback
import cv2
import os
from lane_detection.image_processing.eagelView import BaseLinkTransformation

from lane_detection.image_processing.lane_object import LaneContourHandler

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Quaternion, PoseArray, PoseStamped, Point
from lanelet_msgs.msg import Lanelet,LineString
from cv_bridge import CvBridge
from lane_detection.image_processing.color_filtering import  ColorRange, Color_Filter_Handler
from lane_detection.image_processing.region_of_interest_classes import Roi_Handler
import numpy as np 
from rclpy.logging import LoggingSeverity
import time
from lane_detection.image_params import image_lanelet
import math
import tf2_ros

from lane_detection.image_processing.filter_lane_points import Filter_Through_Lines



class LaneDetectionHandler(Node):
    def __init__(self, 
                 x_off_set_pixel, 
                 y_off_set_pixel,pixel_to_meter_x, 
                 pixel_to_meter_y, kernel_size, publish_data, 
                 subscription_data, call_back_information,
                 show_image,
                 logger_show,
                 points_per_lane,
                 color_ranges = [ColorRange(0, 0, 100, 255, 255, 255)]):
        
        self.color_ranges = color_ranges
        self.call_back_information = call_back_information
        self.color_handler = Color_Filter_Handler(color_ranges,(5,5),(3,3),(1,1))
        self.eagle_view = None
        self.image_show = show_image,
        self.logger_show = logger_show,
        self.points_per_lane = points_per_lane
        self.flag_for_eagle_view = True

        
        super().__init__('subscriber_and_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.publisher = self.create_publisher(
             publish_data['msg_type'],
             publish_data['topic'],
             publish_data['queue_size']) 
 

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/citicar/camera/camera_info",  
            self.callback_camera_info,
            subscription_data['queue_size']
            )
        
        self.subscription = self.create_subscription(
            subscription_data['msg_type'],
            subscription_data['topic'],
            self.callback_get_lane,
            subscription_data['queue_size']
            )
       
       


    def draw_offsets(self,img):
        """
        Draw offset lines and labels on an image.

        This function draws horizontal and vertical lines at regular intervals on the image 
        labels them with their
        corresponding offsets.

        Args:
            img: The image on which to draw the offset lines and labels.

        Returns:
            None: The lines and labels are added to the input image.
        """
        if self.eagle_view is not None:
            i = 0
            while (i * self.eagle_view.pixel_per_meter ) < img.shape[0]:
                horizontal_line_y = img.shape[0] - (int)(i * self.eagle_view.pixel_per_meter)
                cv2.line(
                    img,
                    (0,horizontal_line_y),
                    (img.shape[1],horizontal_line_y),
                    (255,255,255),
                    3
                    )
                
                cv2.putText(
                    img,
                    str(i)  + "X Value", 
                    (10,horizontal_line_y+1),
                    1,
                    2,
                    (255,255,255),
                    2,
                    cv2.LINE_AA
                    )
                i = i + 1
            
            i = 0
            while (i * self.eagle_view.pixel_per_meter ) < img.shape[1]: 
                horizontal_line_x = img.shape[1] -  (int)(i * self.eagle_view.pixel_per_meter )
                
                cv2.line(img,
                        (horizontal_line_x,0),
                        (horizontal_line_x,img.shape[0]),
                        (255,255,255),
                        3
                        )
                
                cv2.putText(img,
                            f"Y value {i}", 
                            (horizontal_line_x,50),
                            1,
                            2,
                            (255,255,255),
                            2,
                            cv2.LINE_AA
                            )
                i = i + 1  
        return img

        
    def publish(self) -> None:
        return
        data=Lanelet()
        for point in self.lan_con_handler.dic_l:
            x, y = point
            #data.left_boundary.line_string.append(self.create_point(x,y))
            #data.right_boundary.line_string.append(self.reate_point(float(i * 10),float(20),float(0)))
        
        for point in self.lan_con_handler.dic_r:
            x, y = point
            data.right_lane.location_vector.append(self.create_point(x,y))
        for i in range(len(self.lan_con_handler.middle_line_points) - 1):   
            p1 = self.lan_con_handler.middle_line_points[i]
            data.middle_lane.location_vector.append(self.create_point(p1[0],p1[0]))           
        self.publisher.publish(data)

    

    def start_obj(self, thresh, image):
        #self.get_logger().log("start with the actualy lane detection" , LoggingSeverity.INFO)
        """
        Start object detection and draw lane lines on the input image.

        This function initializes a LaneContourHandler and processes the image to detect and draw lane lines.

        Args:
            thresh: Thresholded binary image containing lane information.
            image: Input image on which to draw the detected lane lines.

        Returns:
            ndarray: Image with detected lane lines drawn on it.
        """


        self.lan_con_handler = LaneContourHandler(thresh=thresh,pixel_pro_meter=self.eagle_view.pixel_per_meter,
                                                  allowed_distance_forward=self.eagle_view.allowed_distance_forward,
                                                  allowed_distance_sideways=self.eagle_view.allowed_distance_sideways)
        #self.get_logger().log("finish the detection part" , LoggingSeverity.INFO)

        
        if False:
            data=Lanelet()
            publish_pre =  PrepareForPublisher(
                    data,
                    self.lan_con_handler.left_line_countours, 
                    self.lan_con_handler.right_line_countours,
                    self.lan_con_handler.middle_line_points, 
                    self.points_per_lane)
            self.publisher.publish(publish_pre.data)
           
        if (self.image_show):
            if False:
                for con in self.lan_con_handler.middle_line_countours:
                    cv2.drawContours(image, [con.contour], 0, (0, 0, 255), 2)
                    continue

            if True:
                i = 0
                for r in self.lan_con_handler.left_line_countours: 
                    
                   
                    cv2.drawContours(image, [r.contour], -1, color=(0, 255, 0), thickness=2)
                
                    cv2.putText(
                        image,
                        "Left(" + str(i)  + ")={" + str(r.cx) + "/" + str(r.cy) + "}",
                        (r.cx,r.cy),
                        1,
                        2,
                        (255,255,255),
                        1,
                        cv2.LINE_AA
                        )
                    cv2.putText(
                        image,
                        "Max_Y{" + str(r.x_value_highest_row) + "/" + str(r.y_value_highest_row) + "}",  
                        (r.cx,r.cy + 30),
                        1,
                        2,
                        (255,255,255),
                        1,
                        cv2.LINE_AA
                        )


                    i = i + 1

                i = 0
               
                for r in self.lan_con_handler.right_line_countours:
                    
                    cv2.drawContours(image, [r.contour], -1, color=(255, 0, 0), thickness=2)
                    
                    cv2.putText(
                        image,
                        "Right(" + str(i)  + ")center{" + str(r.cx) +"/" + str(r.cy) + "}Max_Y{" + str(r.x_value_highest_row)+ "/" + str(r.y_value_highest_row) + "}",   
                        (r.cx,r.cy),
                        1,
                        2,
                        (255,255,255),
                        1,
                        cv2.LINE_AA
                        )
                    i = i + 1
               
                #print("length " + str(len(self.lan_con_handler.split_contours)))
               # self.split_contours   
                i = 0
                
                

                for c in self.lan_con_handler.other_countours:
                    
                    #cv2.drawContours(image, [c.contour], -1, color=(255, 255, 255), thickness=1)
                    cv2.putText(
                        image,
                        c.debug_message_side_line,
                        (c.cx,c.cy),
                        1,
                        1,
                        (255,255,255),
                        1,
                        cv2.LINE_AA
                        )
                    i = i + 1    

                i = 0
                for r in self.lan_con_handler.split_contours:
                  #  print(r)
                  #  print(r.shape)
                    cv2.drawContours(image,  [r], -1, color=(i * 0, i * 0, i * 0), thickness=3)
                    
                    i = i + 1 
                  
                for r in self.lan_con_handler.stop_line:
                  #  print(r)
                  #  
                    cv2.drawContours(image,  [r], -1, color=(255,100,0), thickness=3)
                    
                    i = i + 1 


            if False:
                for point in self.lan_con_handler.dic_r_2:
                    x, y = point
                    cv2.circle(image, (int(x), int(y)), 1, (255, 0, 0), 3)
                    # cv2.circle(image, (x, y), 1, (255, 0, 0), -1)  # (0, 0, 255) corresponds to the color red

                for point in self.lan_con_handler.dic_l_2:
                    x, y = point
                    cv2.circle(image, (int(x), int(y)), 1, (0, 255, 0), 3)

            if False:
                for point in self.lan_con_handler.dic_r:
                    x, y = point
                    cv2.circle(image, (int(x), int(y)), 1, (255, 0, 0), 3)

                for point in self.lan_con_handler.dic_l:
                    x, y = point
                    cv2.circle(image, (int(x), int(y)), 1, (0, 255, 0), 3)
            
           

            if True:
                for i in range(len(self.lan_con_handler.middle_line_points) - 1):
                    
                    p1 = self.lan_con_handler.middle_line_points[i]   
                    p2 = self.lan_con_handler.middle_line_points[i + 1]
                    
                    cv2.line(image, [int(p1[0]), int(p1[1])], [int(p2[0]), int(p2[1])], (0, 0, 255), 5)   
        return image


    def start(self,image):
        #self.get_logger().log("start with the image_mask and view_transform" , LoggingSeverity.INFO)
        
        if self.eagle_view is None:
            self.get_logger().warn("warten auf Kamera Info..")
            return
        start_time = time.time() 
      
        mask = self.roi.get_mask(image)
        # Apply bitwise AND operation between the mask and the gray image
        masked_image_roi = cv2.bitwise_and(image, image, mask=mask)
       
        
        
        eagle_image_clean = self.eagle_view.transform_image(image)

        
        image = self.eagle_view.transform_image(masked_image_roi)
        masked_image = cv2.bitwise_and(image, image, image)
        
        

        mask = self.color_handler.filter_color(masked_image)
        mask = self.color_handler.morphology_filter()
        #mask = cv2.GaussianBlur(mask,(13,13),0)
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        thresh = self.color_handler.filter_dilate()

        thresh_image = self.color_handler.get_tresh_image(thresh)
        

        first_stacked_image =  np.hstack((cv2.cvtColor(masked_image_roi, cv2.COLOR_BGR2RGB)
                                          ,thresh_image))

        black_image = eagle_image_clean.copy()
        eagle_image = self.start_obj(thresh,eagle_image_clean)
        #black_image = np.zeros((self.eagle_view.pinhole_camera_model.height, self.eagle_view.pinhole_camera_model.width, 3), dtype=np.uint8)
        
        filter_Through_Lines = Filter_Through_Lines(self.eagle_view,black_image,self.lan_con_handler.left_line_countours,self.lan_con_handler.right_line_countours,self.lan_con_handler.middle_line_countours,self.lan_con_handler.pixel_pro_meter,player_point=self.lan_con_handler.player_point)
        
        self.publisher.publish(filter_Through_Lines.lanelet)

        if (self.image_show):
            
            image = self.eagle_view.reverse_transform_image(eagle_image)
            elapsed_time = time.time() - start_time
            fps = 1/elapsed_time
            fps_text = f"FPS: {fps:.2f}/ms{elapsed_time:.4f}"
            

            right_top_with = self.eagle_view.reverse_transform_image(filter_Through_Lines.image)

            cv2.putText(right_top_with,fps_text,(10,50),1,4,255,2)

            #stackedImage = np.hstack((cv2.cvtColor(image, cv2.COLOR_BGR2RGB),

            stackedImage = np.hstack(
                                    (right_top_with,
                                                                eagle_image))
            final_image = np.vstack((stackedImage,first_stacked_image))
            display_image_half_size(final_image)
        #self.get_logger().log("Display image" , LoggingSeverity.INFO)
        return 

    def callback_camera_info(self, msg):
        """
        Callback function for receiving CameraInfo messages.

        This function extracts the necessary information from the CameraInfo message and initializes the EagleView.

        Args:
            msg: CameraInfo message containing camera calibration information.

        Returns:
            None
        """
        if self.flag_for_eagle_view:
            try:
            # Extract the required information from the CameraInfo message
                
                self.tf_buffer.can_transform(source_frame="camera_link", target_frame="base_link", time=rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.01))

                # Erhalten Sie die Transformation von der Kamera zum Weltkoordinatensystem base_link
                transform_stamped = self.tf_buffer.lookup_transform(source_frame="camera_link", target_frame="base_link", time=rclpy.duration.Duration(),timeout=rclpy.duration.Duration(seconds=0.01))     
                self.eagle_view = BaseLinkTransformation(msg,transform_stamped)
                self.flag_for_eagle_view = False
                self.roi = Roi_Handler(pixel_per_meter=self.eagle_view.pixel_per_meter,heigth=self.eagle_view.pinhole_camera_model.height,width=self.eagle_view.pinhole_camera_model.width)
                self.get_logger().log("created a eagle view object" , LoggingSeverity.INFO)
            except tf2_ros.LookupException as e:
                print("Fehler beim Lookup der Transformation: " + str(e))
            
            except tf2_ros.ConnectivityException as e:
                print("Fehler beim Aufbau der Transformationsverbindung: " + str(e))
                
            except tf2_ros.ExtrapolationException as e:
                print("Fehler bei der Extrapolation der Transformation: " + str(e))
        





    def test_world_coordinates(self,cv_image):
        pixel_points = [(
        int(1440/4), int(1080/2)), 
        (int(1440/1.3), int(1080/1.2)),
        (int(1440/1.3), int(1080/2.0)),  
        (int(1440/1.3), int(1080/1.5)),
        (int(1440/1.6), int(1080/3.0)),
        (int(1440/1.6), int(1080/2.0)),  
        (int(1440/1.6), int(1080/0.8)),
        (int(1440/1.2), int(1080/1.3)),
        (int(1440/2.3), int(1080/2.0)),  
        (int(1440/1.2), int(1080/1.5)),
        (int(1440/2.6), int(1080/1.8)),
        (int(1440/1.6), int(1080/2.0)),  
        (int(1440/1.6), int(1080/0.8)),  
        (int(1440/2), int(1080/2))]

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), 
                    (255, 255, 0),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
                    (255, 255, 0),(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]


        for point, color in zip(pixel_points, colors):
            cv2.circle(cv_image, center=point, radius=10, color=color, thickness=-1)  # radius-10 für einen vollen Kreis
            calc_cor = self.eagle_view.get_worldcordinate_for_pixel(point)


            text = "x={:.1f}, y={:.1f}, z={:.1f}".format(calc_cor[0], calc_cor[1], calc_cor[2])
            
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1
            text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
            text_x = point[0] + 15  # Shift the text to the right of the circle
            text_y = point[1] + text_size[1] // 2  # Center the text vertically
            cv2.putText(cv_image, text, (text_x, text_y), font, font_scale, color, font_thickness)
        display_image_in_one_half_size(image=cv_image)
        


    def callback_get_lane(self, msg):
        """
        Callback function for receiving lane information messages.

        This function processes the received lane information message, converts it to an image, and starts the lane detection process.

        Args:
            msg: Lane information message containing lane data.

        Returns:
            None
        """
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            self.start(cv_image)


        
            #display_image_in_one_half_size(image=cv_image)
        except Exception as e:
            #self.get_logger().warn("Exception in image_callback in the Subscriber:\n" + str(e))
            traceback.print_exc()
        

def display_image_half_size(image):
    """
    Display an image at half its original size.

    This function resizes the input image to half its original dimensions and displays it using OpenCV.

    Args:
        image: Input image to be displayed.

    Returns:
        None
    """
    # Get the original image dimensions
    height, width = image.shape[:2]

    # Calculate the new dimensions for resizing
    new_width = int(width / 2)
    new_height = int(height / 2)

    # Resize the image to half its size
    resized_image = cv2.resize(image, (new_width, new_height))

    # Display the resized image
    cv2.imshow("Resized Image", resized_image)
    cv2.waitKey(1)

def display_image_in_one_half_size(image):
    """
    Display an image at one and a half times its original size.

    This function resizes the input image to one and a half times its original dimensions and displays it using OpenCV.

    Args:
        image: Input image to be displayed.

    Returns:
        None
    """
    # Get the original image dimensions
    height, width = image.shape[:2]

    # Calculate the new dimensions for resizing
    new_width = int(width * 0.66667)
    new_height = int(height * 0.66667)

    # Resize the image to one and a half times its size
    resized_image = cv2.resize(image, (new_width, new_height))

    # Display the resized image
    cv2.imshow("Resized Image", resized_image)
    cv2.waitKey(1)

def display_image_(image):
    """
    Display an image at one and a half times its original size.

    This function resizes the input image to one and a half times its original dimensions and displays it using OpenCV.

    Args:
        image: Input image to be displayed.

    Returns:
        None
    """
    # Get the original image dimensions
    height, width = image.shape[:2]

    # Calculate the new dimensions for resizing
    new_width = int(width * 1)
    new_height = int(height *1)

    # Resize the image to one and a half times its size
    resized_image = cv2.resize(image, (new_width, new_height))

    # Display the resized image
    cv2.imshow("Resized Image", resized_image)
    cv2.waitKey(1)



def main(args=None):
    rclpy.init()
    node=rclpy.create_node('PozyxPublisher')
    param_listener=image_lanelet.ParamListener(node)
    params = param_listener.get_params()
    publisher_data = {
        'msg_type': Lanelet,                      # The message type for the publisher
        'topic': params.topic_publish,                  # The topic to publish to
        'queue_size': params.queue_size_publish,        # The size of the message queue
    }

    subscription_data = {
        'msg_type': Image,                        # The message type for the subscription
        'topic': params.topic_subscribe,            # The topic to subscribe to
        'queue_size': params.queue_size_subscribe,  # The size of the message queue
    }

    call_back_information = {
        'frame_id' : params.frame_id,
        'show_option' : params.show,
    }
    lane_let = LaneDetectionHandler(
        params.x_off_set_pixel,
        params.y_off_set_pixel,
        params.pixel_to_meter_x,
        params.pixel_to_meter_y,
        (params.kernel,params.kernel),
        publish_data=publisher_data,
        subscription_data=subscription_data,
        call_back_information=call_back_information,
        show_image=params.image_show,
        logger_show=params.logger_show,
        points_per_lane=params.points_per_lane

    )
    rclpy.spin(lane_let)
    lane_let.destroy_node()

if __name__ == '__main__':
    main()