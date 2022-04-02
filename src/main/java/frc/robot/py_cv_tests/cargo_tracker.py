# PROGRAM: Finds the closest cargo of the desired color
from queue import Queue
import cv2 as cv, numpy as np, json, threading, time
from networktables import NetworkTables

tasks = Queue()
returnedFramesQueue = Queue()
threadsBeingUsed = 0

# Create color ranges for red and blue
# Red_left
RED_LOWER = np.array([0, 150, 15], dtype ="uint8")
RED_UPPER = np.array([9, 255, 255], dtype ="uint8")
RED2_LOWER = np.array([165, 150, 15], dtype ="uint8")
RED2_UPPER = np.array([180, 255, 255], dtype ="uint8")
# Blue
BLUE_LOWER = np.array([100, 100, 40], dtype = "uint8")
BLUE_UPPER = np.array([130, 255, 255], dtype = "uint8")

class FrameContainer:
    def __init__(self, frame):
        self.frame = frame

def detection_thread(*args):
    global returnedFramesQueue, threadsBeingUsed
    frame = args[0]
    # Scale down frame by 0.5 to speed up computation
    frame = cv.resize(frame, (frame.shape[1]//2,frame.shape[0]//2), interpolation=cv.INTER_LINEAR)
    # List to store ball info
    ball_array = []
    # Convert to HSV
    kVal = 43
    hsv = cv.cvtColor(cv.GaussianBlur(frame, (kVal, kVal), 0), cv.COLOR_BGR2HSV)
    # Find the red and blue contours
    # red_mask = cv.inRange(hsv, RED_LOWER, RED_UPPER)
    red_mask = cv.bitwise_or(cv.inRange(hsv, RED_LOWER, RED_UPPER), cv.inRange(hsv, RED2_LOWER, RED2_UPPER))
    blue_mask = cv.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    # Get circles
    red_circles = cv.HoughCircles(red_mask, cv.HOUGH_GRADIENT,
                                  dp=1.5,
                                  minDist=300,
                                  param1=100,
                                  param2=30,
                                  minRadius=10,
                                  maxRadius=400)
    blue_circles = cv.HoughCircles(blue_mask, cv.HOUGH_GRADIENT,
                                  dp=1.5,
                                  minDist=300,
                                  param1=100,
                                  param2=30,
                                  minRadius=10,
                                  maxRadius=400)
    if red_circles is not None:
        red_circles = np.uint16(np.around(red_circles))
        for circle in red_circles[0, :]:
            cv.circle(frame, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
            ball_array.append({
                'id': len(ball_array),
                'color': 'red',
                'centerX': float(circle[1]),
                'centerY': float(circle[0]),
                'radius': float(circle[2])})
    if blue_circles is not None:
        blue_circles = np.uint16(np.around(blue_circles))
        for circle in blue_circles[0, :]:
            cv.circle(frame, (circle[0], circle[1]), circle[2], (255, 0, 0), 2)
            ball_array.append({
                'id': len(ball_array),
                'color': 'blue',
                'centerX': float(circle[1]),
                'centerY': float(circle[0]),
                'radius': float(circle[2])})
    # Post to network tables
    jsonData = json.dumps(ball_array)
    # print(jsonData)
    # TODO: Fix the below code
    NetworkTables.initialize(server='roborio-177-frc.local')
    sd = NetworkTables.getTable('pyVision')
    sd.putString('jsonData', jsonData)

    # Add desired frames to queue for debug viewing
    returnedFramesQueue.put((frame, red_mask, blue_mask))
    threadsBeingUsed -= 1

detection = threading.Thread(target=detection_thread, name="detection_thread")
detection.start()

fps = 30
prev = 0

if __name__ == '__main__':
    # Open the camera
    cap = cv.VideoCapture(2)
    warningNotDelivered = True

    while True:
        # print("Currently using", threadsBeingUsed, "threads")
        # Put next available task on thread
        # Lets start with max 3 (even though we can do
        # 4 with Raspberry Pi) just to safety-test
        time_elapsed = time.time() - prev
        if threadsBeingUsed <= 3 and not tasks.empty():
            threadsBeingUsed += 1
            tasks.get().start()

        ret, frame = cap.read()

        while time_elapsed <= 1.0/fps:
            time_elapsed = time.time() - prev
        prev = time.time()

        if not ret:
            print("Empty frame; continuing...")
            continue

        tasks.put_nowait(threading.Thread(target=detection_thread, args=(frame,)))

        # For safety reasons
        if tasks.qsize() >= 10000 and warningNotDelivered:
            print("Warning: >=10000 tasks in the queue")
            warningNotDelivered = False
        if tasks.qsize() < 10000:
            warningNotDelivered = True

        # Get most recently processed frames
        if not returnedFramesQueue.empty():
            # print("Got frame")
            frame, red_mask, blue_mask = returnedFramesQueue.get()
            cv.imshow("Frame", frame)
            cv.imshow("Blue mask", blue_mask)
            cv.imshow("Red mask", red_mask)

        # Exit on 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Close the camera
    cap.release()
    cv.destroyAllWindows()










