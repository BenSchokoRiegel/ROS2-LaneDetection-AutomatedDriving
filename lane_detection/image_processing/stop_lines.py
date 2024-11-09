import cv2
import np as Numpy


class StopLine:
    def __init__(self, contours,stop_Line_length, stop_Line_hight,schwellenwert):
        self.stop_line = []
        self.stop_text = []

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
                
                lines_longer_then_raing = [stop_Line_length > cv2.norm(approx[i-1], approx[i])/ self.pixel_pro_meter ]
                lines_longer_than_range = [i for i in range(1, len(approx)) if stop_Line_length > cv2.norm(approx[i-1], approx[i]) / self.pixel_pro_meter]

                perfect_length = []
                longer_than = []
                
                cont_obj.debug_message_side_line = str(len(approx))
                self.other_countours.append(cont_obj)

                for i in range(1, len(approx)):
                    # Calculate the length of the approximated line
                    #  self.stop_line.append(approx[i-1:i+1])
                    length = cv2.norm(approx[i-1], approx[i])/ self.pixel_pro_meter
                    #   print(length)
                    if len(approx) > 4 and length > 0.23 and length < 0.28:
                        perfect_length.append(approx[i-1], approx[i])    

                    if length > 0.20:
                        longer_than.append(approx[i-1], approx[i])   




                       
                for perfect in perfect_length:
                    pa = perfect[0]
                    pb = perfect[1]
                    mx,my = mitte_zwischen_punkten(pa[0],pa[1],pb[0],pb[1])

                    for longer in longer_than:
                       
                        x1, y1 = longer[0]
                        x2, y2 =  longer[1]

                        # Berechne den Abstand zwischen den beiden Punkten
                        abstand = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)/self.pixel_pro_meter

                        # Überprüfe, ob der Abstand größer als die gegebene Länge ist
            
                        # Berechne den Punkt auf der Linie, der die gegebene Länge vom ersten Punkt entfernt ist
                        t = (stop_Line_length / 2) / abstand
                        x_punkt = (1 - t) * x1 + t * x2
                        y_punkt = (1 - t) * y1 + t * y2

                        abstand_zu_mitte = np.sqrt((mx - x_punkt)**2 + (my - y_punkt)**2)/self.pixel_pro_meter
                        

                        if np.abs(abstand_zu_mitte > schwellenwert) 


                        self.stop_Line_hight


        def mitte_zwischen_punkten(self,x1, y1, x2, y2):
            mittelpunkt_x = (x1 + x2) / 2
            mittelpunkt_y = (y1 + y2) / 2
            return mittelpunkt_x, mittelpunkt_y  

















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
                        