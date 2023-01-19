# PROGRAM: Finds the closest cargo of the desired color
import cv2 as cv, numpy as np, json
from networktables import NetworkTables

# Create color ranges for red and blue
# Red_left\
RED_LOWER = np.array([0, 130, 20], dtype ="uint8")
RED_UPPER = np.array([7, 255, 255], dtype ="uint8")
# # Red_right
# RED_RIGHT_LOWER = np.array([177, 100, 40], dtype = "uint8")
# RED_RIGHT_UPPER = np.array([180, 255, 255], dtype = "uint8")
# Blue
BLUE_LOWER = np.array([100, 100, 40], dtype = "uint8")
BLUE_UPPER = np.array([130, 255, 255], dtype = "uint8")

# Open the camera
cap = cv.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Empty frame")
        continue
    # List to store ball info
    ball_array = []
    # Convert to HSV
    kVal = 27
    hsv = cv.cvtColor(cv.GaussianBlur(frame, (kVal, kVal), 0), cv.COLOR_BGR2HSV)
    # Find the red and blue contours
    red_mask = cv.inRange(hsv, RED_LOWER, RED_UPPER)
    blue_mask = cv.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    red_mask = cv.Canny(red_mask, 40, 100)
    blue_mask = cv.Canny(blue_mask, 40, 100)
    # # Get circles
    # red_circles = cv.HoughCircles(red_mask, cv.HOUGH_GRADIENT, dp=1.5, minDist=300, param1=100, param2=30, minRadius=80, maxRadius=400)
    # blue_circles = cv.HoughCircles(blue_mask, cv.HOUGH_GRADIENT, dp=1.5, minDist=300, param1=100, param2=30, minRadius=80, maxRadius=400)
    # if red_circles is not None:
    #     red_circles = np.uint16(np.around(red_circles))
    #     for circle in red_circles[0, :]:
    #         cv.circle(frame, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
    #         ball_array.append({
    #             'id': len(ball_array),
    #             'color': 'red',
    #             'centerX': float(circle[1]),
    #             'centerY': float(circle[0]),
    #             'radius': float(circle[2])})
    # if blue_circles is not None:
    #     blue_circles = np.uint16(np.around(blue_circles))
    #     for circle in blue_circles[0, :]:
    #         cv.circle(frame, (circle[0], circle[1]), circle[2], (255, 0, 0), 2)
    #         ball_array.append({
    #             'id': len(ball_array),
    #             'color': 'blue',
    #             'centerX': float(circle[1]),
    #             'centerY': float(circle[0]),
    #             'radius': float(circle[2])})

    # # Show the frame
    cv.imshow("Frame", frame)
    cv.imshow("Blue mask", blue_mask)
    cv.imshow("Red mask", red_mask)

    # # Post to network tables
    # jsonData = json.dumps(ball_array)
    # print(jsonData)
    # # TODO: Fix the below code
    # NetworkTables.initialize(server='roborio-177-frc.local')
    # sd = NetworkTables.getTable('pyVision')
    # sd.putString('jsonData', jsonData)
    # # Exit on 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Close the camera
cap.release()
cv.destroyAllWindows()










