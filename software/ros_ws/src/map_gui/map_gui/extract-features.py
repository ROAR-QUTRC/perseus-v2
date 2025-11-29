""" Identify map features based on colour """

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

""" Import OpenCV and numpy libraries """
import cv2
import numpy as np
import math


class ExtractFeatures(Node):
    def __init__(self):
        """I'll figure out what to put here later"""
        super().__init__("extract_features")

    """This should probably be split up"""
    def everything_else(self):
        font = cv2.FONT_HERSHEY_COMPLEX

        """ Read the image """
        map = cv2.imread("ARCh_2025_Autonomous_map.png")

        """ Extract dimensions of the image """
        length, width, height = map.shape

        """ Convert image to image HSV """
        hsv = cv2.cvtColor(map, cv2.COLOR_BGR2HSV)


        """ Define lower and upper bound HSV values """ 
        #Outerperimeter - currently green - TO BE CHANGED AS NECESSARY
        perimeter_lower = np.array([50,40, 0])
        perimeter_upper = np.array([70, 255, 255])

        #Waypoints - currently green-blue - TO BE CHANGED AS NECESSARY
        waypoints_lower = np.array([70, 150, 170])
        waypoints_upper = np.array([90, 255, 255])

        #x-starting point - currently orange - TO BE CHANGED AS NECESSARY
        x_lower = np.array([5, 100, 255])
        x_upper = np.array([30, 255, 255])

        #Grid - currently red - TO BE CHANGED AS NECESSARY
        grid_lower = np.array([10, 0, 0])
        grid_upper = np.array([100, 255, 255])

        """ Define masks for detecting color """
        perimeter_mask = cv2.inRange(hsv, perimeter_lower, perimeter_upper)
        waypoints_mask = cv2.inRange(hsv, waypoints_lower, waypoints_upper)
        x_mask = cv2.inRange(hsv, x_lower, x_upper)
        grid_mask = cv2.inRange(hsv, grid_lower, grid_upper)

        """Turn mask back into image so that it can be seperated into colour and gray scale versions"""
        #There is probably a way to not have to do this
        cv2.imwrite("perimeter_mask.png", perimeter_mask)
        cv2.imwrite("waypoints_mask.png", waypoints_mask)
        cv2.imwrite("x_mask.png", x_mask)
        cv2.imwrite("grid_mask.png", grid_mask)


        """ Define colour and grayscale version of masks """
        perimeter_colour = cv2.imread("perimeter_mask.png", cv2.IMREAD_COLOR)
        perimeter_grayscale = cv2.imread("perimeter_mask.png", cv2.IMREAD_GRAYSCALE)

        waypoints_colour = cv2.imread("waypoints_mask.png", cv2.IMREAD_COLOR)
        waypoints_grayscale = cv2.imread("waypoints_mask.png", cv2.IMREAD_GRAYSCALE)

        x_colour = cv2.imread("x_mask.png", cv2.IMREAD_COLOR)
        x_grayscale = cv2.imread("x_mask.png", cv2.IMREAD_GRAYSCALE)

        grid_colour = cv2.imread("grid_mask.png", cv2.IMREAD_COLOR)
        grid_grayscale = cv2.imread("grid_mask.png", cv2.IMREAD_GRAYSCALE)

        """ Binarize masks """
        _, perimeter_threshold = cv2.threshold(perimeter_grayscale, 110, 255, cv2.THRESH_BINARY)
        _, waypoints_threshold = cv2.threshold(waypoints_grayscale, 110, 255, cv2.THRESH_BINARY)
        _, x_threshold = cv2.threshold(x_grayscale, 110, 255, cv2.THRESH_BINARY)
        _, grid_threshold = cv2.threshold(grid_grayscale, 110, 255, cv2.THRESH_BINARY)

        """ Find contours """
        perimeter_contours, _ = cv2.findContours(perimeter_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        waypoints_contours, _ = cv2.findContours(waypoints_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        x_contours, _ = cv2.findContours(x_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        grid_contours, _ = cv2.findContours(grid_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        """ Define important coordinates """
        ORIGIN_X = (0,0)
        ORIGIN_Y = (0,0)
        ORIGIN = (0,0)
        PERIMETER_TL = (0,0)
        PERIMETER_TR = (0,0)
        PERIMETER_BL = (0,0)
        PERIMETER_BR = (0,0)
        WAYPOINT_COORDS = []
        ARROW_COORDS = []
        WAYPOINT_ANGLE = []

        """Find perimeter coordinates and write them to the mask and map"""
        if perimeter_contours:
            #Get coordinates of all points in the first detected contour (perimeter)
            first_contour =  perimeter_contours[0]

            #Convert array to list of usable tuples
            first_contour_formatted = []
            for i  in range (0, len(first_contour), 1):
                first_contour_formatted.append(tuple(first_contour[i][0]))

            #Extract extreme points
            top_left = (min(x[0] for x in first_contour_formatted), min(x[1] for x in first_contour_formatted))
            bottom_left = (min(x[0] for x in first_contour_formatted), max(x[1] for x in first_contour_formatted))
            top_right = (max(x[0] for x in first_contour_formatted), min(x[1] for x in first_contour_formatted))
            bottom_right = (max(x[0] for x in first_contour_formatted), max(x[1] for x in first_contour_formatted))

            #Assign coordinates to global variables
            PERIMETER_TL = top_left
            PERIMETER_BL = bottom_left
            PERIMETER_TR = top_right
            PERIMETER_BR = bottom_right

            #Print coordinates on mask
            top_edges = [top_left, top_right]
            bottom_edges = [bottom_right, bottom_left]

            for edge in top_edges:
                # cv2.putText(map, f"({str(edge[0])}, {str(edge[1])})", (edge[0]-20, edge[1]-10), font, 0.5, (0, 0, 255))
                # cv2.circle(map, (edge[0], edge[1]), radius=3, color=(0, 0, 255), thickness=-1)
                cv2.putText(perimeter_colour, f"({str(edge[0])}, {str(edge[1])})", (edge[0]-20, edge[1]-10), font, 0.5, (0, 0, 255))
                cv2.circle(perimeter_colour, (edge[0], edge[1]), radius=3, color=(0, 0, 255), thickness=-1)

            for edge in bottom_edges:
                # cv2.putText(map, f"({str(edge[0])}, {str(edge[1])})", (edge[0]-20, edge[1]+20), font, 0.5, (0, 0, 255))
                # cv2.circle(map, (edge[0], edge[1]), radius=3, color=(0, 0, 255), thickness=-1)
                cv2.putText(perimeter_colour, f"({str(edge[0])}, {str(edge[1])})", (edge[0]-20, edge[1]+20), font, 0.5, (0, 0, 255))
                cv2.circle(perimeter_colour, (edge[0], edge[1]), radius=3, color=(0, 0, 255), thickness=-1)

            #Get coordinate of y arrow
            arrow_contour = perimeter_contours[5]

            #Convert array to list of usable tuples
            arrow_contour_formatted = []
            for i  in range (0, len(arrow_contour), 1):
                arrow_contour_formatted.append(tuple(arrow_contour[i][0]))
            
            #Extract y valyes to find median
            arrow_contour_formatted_y = []
            for i in range (0, len(arrow_contour_formatted), 1):
                arrow_contour_formatted_y.append(arrow_contour_formatted[i][1])

            #Extract rightmost point
            right_edge = (max(x[0] for x in arrow_contour_formatted), round(np.median(arrow_contour_formatted_y)))
            ORIGIN_Y = right_edge

            #Print coordinates on mask and map
            # cv2.putText(map, f"({str(right_edge[0])}, {str(right_edge[1])})", (right_edge[0]-20, right_edge[1]-10), font, 0.5, (0, 0, 255))
            # cv2.circle(map, (right_edge[0], right_edge[1]), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.putText(perimeter_colour, f"({str(right_edge[0])}, {str(right_edge[1])})", (right_edge[0]+20, right_edge[1]), font, 0.5, (0, 0, 255))
            cv2.circle(perimeter_colour, (right_edge[0], right_edge[1]), radius=3, color=(0, 0, 255), thickness=-1)

            #print(f"Extreme points: Left={top_left}, Right={bottom_right}, Top={top_right}, Bottom={bottom_left}")

        if x_contours:
            #Get coordinates of all points in the firt contour (arrow)
            arrow_contour =  x_contours[0]

            #convert array to list of usable tuples
            arrow_contour_formatted = []
            for i  in range (0, len(arrow_contour), 1):
                arrow_contour_formatted.append(tuple(arrow_contour[i][0]))
            
            #Extract x valyes to find median
            arrow_contour_formatted_x = []
            for i in range (0, len(arrow_contour_formatted), 1):
                arrow_contour_formatted_x.append(arrow_contour_formatted[i][0])

            #Extract rightmost point
            bottom_edge = (round(np.median(arrow_contour_formatted_x)), max(x[1] for x in arrow_contour_formatted))
            ORIGIN_X = bottom_edge

            #Print coordinate on mask
            #cv2.putText(map, f"({str(bottom_edge[0])}, {str(bottom_edge[1])})", (bottom_edge[0]-20, bottom_edge[1]-10), font, 0.5, (0, 0, 255))
            #cv2.circle(map, (bottom_edge[0], bottom_edge[1]), radius=3, color=(0, 0, 255), thickness=-1)
            #cv2.putText(x_colour, f"({str(bottom_edge[0])}, {str(bottom_edge[1])})", (bottom_edge[0]-20, bottom_edge[1]-10), font, 0.5, (0, 0, 255))
            #cv2.circle(x_colour, (bottom_edge[0], bottom_edge[1]), radius=3, color=(0, 0, 255), thickness=-1)

        if waypoints_contours:
            #print(waypoints_contours)
            #Get coordinates of all points in the firt contour (arrow)
            #center_contour =  waypoints_contours[36]
            circle_contours = [0, 14, 36, 55, 62]
            arrow_contours = [12, 13, 29, 70, 71]

            #convert array to list of usable tuples
            
            for i  in range (0, len(circle_contours), 1):
                contour_formatted = []
                for j  in range (0, len(waypoints_contours[circle_contours[i]]), 1):
                    contour_formatted.append(tuple(waypoints_contours[circle_contours[i]][j][0]))

                    #print(contour_formatted)
                    contour_formatted_x = []
                    contour_formatted_y = []
                    for k in range (0, len(contour_formatted), 1):
                        contour_formatted_x.append(contour_formatted[k][0])
                        contour_formatted_y.append(contour_formatted[k][1])

                #Extract middle point
                middle = (round((max(x[0] for x in contour_formatted)+min(x[0] for x in contour_formatted))/2), round((max(x[1] for x in contour_formatted)+min(x[1] for x in contour_formatted))/2))
                WAYPOINT_COORDS.append(middle)

                cv2.putText(waypoints_colour, f"({str(middle[0])}, {str(middle[1])})", (middle[0]-100, middle[1]), font, 0.5, (0, 0, 255))
                cv2.circle(waypoints_colour, (middle[0], middle[1]), radius=3, color=(0, 0, 255), thickness=-1)
                    
            for i  in range (0, len(arrow_contours), 1):
                contour_formatted = []
                for j  in range (0, len(waypoints_contours[arrow_contours[i]]), 1):
                    contour_formatted.append(tuple(waypoints_contours[arrow_contours[i]][j][0]))

                    #print(contour_formatted)
                    contour_formatted_x = []
                    contour_formatted_y = []
                    for k in range (0, len(contour_formatted), 1):
                        contour_formatted_x.append(contour_formatted[k][0])
                        contour_formatted_y.append(contour_formatted[k][1])

                #Extract middle point
                middle = (round((max(x[0] for x in contour_formatted)+min(x[0] for x in contour_formatted))/2), round((max(x[1] for x in contour_formatted)+min(x[1] for x in contour_formatted))/2))
                ARROW_COORDS.append(middle)

                #cv2.putText(waypoints_colour, f"({str(middle[0])}, {str(middle[1])})", (middle[0]-100, middle[1]), font, 0.5, (0, 0, 255))
                #cv2.circle(waypoints_colour, (middle[0], middle[1]), radius=3, color=(0, 0, 255), thickness=-1)

            for i in range (0, len(WAYPOINT_COORDS)):
                delta_x = WAYPOINT_COORDS[i][0] - ARROW_COORDS[i][0]
                delta_y = WAYPOINT_COORDS[i][1] - ARROW_COORDS[i][1]
                print(delta_x, delta_y)
            
                if (delta_x == 0): 
                    if (WAYPOINT_COORDS[i][1] <= ARROW_COORDS[i][1]):
                            WAYPOINT_ANGLE.append(np.pi)
                    elif(WAYPOINT_COORDS[i][1] >= ARROW_COORDS[i][1]):
                        WAYPOINT_ANGLE.append(0)
                elif (delta_y == 0):
                    if (WAYPOINT_COORDS[i][0] >= ARROW_COORDS[i][0]):
                        WAYPOINT_ANGLE.append(np.pi/2)
                    elif(WAYPOINT_COORDS[i][0] <= ARROW_COORDS[i][0]):
                        WAYPOINT_ANGLE.append(3*np.pi/2)
                else:
                    if ((delta_x > 0) & (delta_y > 0)):
                        WAYPOINT_ANGLE.append(math.atan(delta_x/delta_y))
                        print(1, WAYPOINT_COORDS[i][0], WAYPOINT_COORDS[i][1])
                    elif((delta_x > 0) & (delta_y < 0)):
                        WAYPOINT_ANGLE.append(3*np.pi/2 + math.atan(delta_x/delta_y))
                        print(2, WAYPOINT_COORDS[i][0], WAYPOINT_COORDS[i][1])
                    elif((delta_x < 0) & (delta_y < 0)):
                        WAYPOINT_ANGLE.append(np.pi + math.atan(delta_x/delta_y))
                        print(4, WAYPOINT_COORDS[i][0], WAYPOINT_COORDS[i][1])
                    elif((delta_x < 0) & (delta_y > 0)):
                        WAYPOINT_ANGLE.append(np.pi/2 + math.atan(delta_x/delta_y))
                        print(3, WAYPOINT_COORDS[i][0], WAYPOINT_COORDS[i][1])

                print(WAYPOINT_ANGLE)

                
            

        #Find origin by taking average of coordinates taken from the x and y arrows
        ORIGIN = tuple(round((a + b) / 2) for a, b in zip(ORIGIN_X, ORIGIN_Y))

        #Print origin coordinate on map
        cv2.putText(map, f"(0,0)", (ORIGIN[0]+10, ORIGIN[1]), font, 0.70, (255, 255, 255))
        cv2.circle(map, (ORIGIN[0], ORIGIN[1]), radius=4, color=(255, 255, 255), thickness=-1)

        #Find coordinates of important locations relative to the origin
        PERIMETER_TL_RELATIVE = (ORIGIN[0] - PERIMETER_TL[0], ORIGIN[1] - PERIMETER_TL[1])
        PERIMETER_TR_RELATIVE = (ORIGIN[0] - PERIMETER_TR[0], ORIGIN[1] - PERIMETER_TR[1])
        PERIMETER_BL_RELATIVE = (ORIGIN[0] - PERIMETER_BL[0], ORIGIN[1] - PERIMETER_BL[1])
        PERIMETER_BR_RELATIVE = (ORIGIN[0] - PERIMETER_BR[0], ORIGIN[1] - PERIMETER_BR[1])

        #Print coordinates of important locations on map
        cv2.putText(map, f"({str(PERIMETER_TL_RELATIVE[1])}, {str(PERIMETER_TL_RELATIVE[0])})", (PERIMETER_TL[0]-20, PERIMETER_TL[1]-10), font, 0.6, (255, 255, 255))
        cv2.circle(map, (PERIMETER_TL[0], PERIMETER_TL[1]), radius=4, color=(255, 255, 255), thickness=-1)
        cv2.putText(map, f"({str(PERIMETER_TR_RELATIVE[1])}, {str(PERIMETER_TR_RELATIVE[0])})", (PERIMETER_TR[0]-120, PERIMETER_TR[1]-10), font, 0.6, (255, 255, 255))
        cv2.circle(map, (PERIMETER_TR[0], PERIMETER_TR[1]), radius=4, color=(255, 255, 255), thickness=-1)
        cv2.putText(map, f"({str(PERIMETER_BL_RELATIVE[1])}, {str(PERIMETER_BL_RELATIVE[0])})", (PERIMETER_BL[0]-20, PERIMETER_BL[1]+20), font, 0.6, (255, 255, 255))
        cv2.circle(map, (PERIMETER_BL[0], PERIMETER_BL[1]), radius=4, color=(255, 255, 255), thickness=-1)
        cv2.putText(map, f"({str(PERIMETER_BR_RELATIVE[1])}, {str(PERIMETER_BR_RELATIVE[0])})", (PERIMETER_BR[0]-120, PERIMETER_BR[1]+20), font, 0.6, (255, 255, 255))
        cv2.circle(map, (PERIMETER_BR[0], PERIMETER_BR[1]), radius=4, color=(255, 255, 255), thickness=-1)

        for i in range(0, len(WAYPOINT_COORDS), 1):
            waypoint_relative = (WAYPOINT_COORDS[i][0] - ORIGIN[0], ORIGIN[1] - WAYPOINT_COORDS[i][1])
            cv2.putText(map, f"({str(waypoint_relative[0])}, {str(waypoint_relative[1])})", (WAYPOINT_COORDS[i][0]-110, WAYPOINT_COORDS[i][1]+30), font, 0.6, (255, 255, 255))
            cv2.putText(map, f"({round(WAYPOINT_ANGLE[i],2)} rad)", (WAYPOINT_COORDS[i][0]-90, WAYPOINT_COORDS[i][1]+60), font, 0.6, (255, 255, 255))
            cv2.circle(map,(WAYPOINT_COORDS[i][0], WAYPOINT_COORDS[i][1]), radius=4, color=(255, 255, 255), thickness=-1)



        # for contour in perimeter_contours:
        #     #Approximate and draw contour
        #     approx = cv2.approxPolyDP(contour, 0.009 * cv2.arcLength(contour, True), True)
        #     #cv2.drawContours(perimeter_colour, [approx], 0, (0, 0, 255), 5)

        #     # Flatten points
        #     n = approx.ravel()
        #     i = 0
        #     for j in n:
        #         if i % 2 == 0:  # x, y coordinates
        #             x, y = n[i], n[i + 1]
        #             coord = f"{x} {y}"
        #             cv2.putText(perimeter_colour, coord, (x, y), font, 0.5, (0, 255, 0))
        #         i += 1

        # for contour in waypoints_contours:
        #     #Approximate and draw contour
        #     approx = cv2.approxPolyDP(contour, 0.009 * cv2.arcLength(contour, True), True)
        #     #cv2.drawContours(perimeter_colour, [approx], 0, (0, 0, 255), 5)

        #     # Flatten points
        #     n = approx.ravel()
        #     i = 0
        #     for j in n:
        #         if i % 2 == 0:  # x, y coordinates
        #             x, y = n[i], n[i + 1]
        #             coord = f"{x} {y}"
        #             cv2.putText(waypoints_colour, coord, (x, y), font, 0.5, (0, 255, 0))
        #             #print("(" + str(x) + ", " +  str(y) + ")")
        #         i += 1

        # Show result
        #cv2.imshow('Perimeter with Coordinates', perimeter_colour)
        #cv2.imshow('X with Coordinates', x_colour)
        #cv2.imshow('Waypoints with Coordinates', waypoints_colour)
        combined_image = cv2.bitwise_or(perimeter_colour, waypoints_colour)
        combined_image = cv2.bitwise_or(x_colour, combined_image)
        cv2.imshow('Combined Coordinates', combined_image)


        # #Grid - DOESNT LINE UP
        # for i in range(0,width+1,25):
        #     for j in range (0, length+1, 50):
        #         cv2.line(map,(i,0),(i,j),(0,0,255),1)
        #         cv2.line(map,(0,i),(j,i),(0,0,255),1)

        #Display map and masks
        cv2.imshow("Autonomous Map", map)
        #cv2.imshow("Grid", grid_mask)
        #cv2.imshow("Mask", mask)
        #cv2.imshow("Mask2", mask2)

        #Make python sleep for unlimited time
        cv2.waitKey(0)


def main(args=None):
    rclpy.init(args=args)
    extract_features = ExtractFeatures()
    rclpy.spin(extract_features)
    extract_features.destroy_node()
    rclpy.shutdown()
