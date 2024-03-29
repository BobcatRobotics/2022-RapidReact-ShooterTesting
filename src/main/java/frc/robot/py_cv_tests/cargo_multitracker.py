# PROGRAM: Finds the closest cargo of the desired color
from queue import Queue
import cv2 as cv, numpy as np, json, threading, time
from networktables import NetworkTables
from cscore import CameraServer


cond = threading.Condition()
notified = [False]

tasks = Queue()
returnedFramesQueue = Queue()
threadsBeingUsed = 0
h = 720
w = 1280
fov = 55

CameraServer.enableLogging()
camera = CameraServer.startAutomaticCapture()
camera.setResolution(w,h)

globalFrame = None

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server='10.01.77.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting to connect NetworkTables")
    if not notified[0]:
        cond.wait()

# Insert your processing code here
print("Connected to NetworkTables!")

# Create color ranges for red and blue
# Red_left
RED_LOWER = np.array([0, 180, 20], dtype ="uint8")
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
    global returnedFramesQueue, threadsBeingUsed, globalFrame
    frame = globalFrame
    if frame.any() == None:
        return

    # Scale down frame by 0.5 to speed up computation
    frame = cv.resize(frame, (frame.shape[1]//2,frame.shape[0]//2), interpolation=cv.INTER_LINEAR)
    width = frame.shape[1]
    height = frame.shape[0]
    sd = NetworkTables.getTable('pyVision')
    colour = sd.getString("teamColor","red")

    # List to store ball info
    ball_array = []
    # Convert to HSV
    kVal = 43
    hsv = cv.cvtColor(cv.GaussianBlur(frame, (kVal, kVal), 0), cv.COLOR_BGR2HSV)
    # Find the red and blue contours
    # Get circles
    red_circles = None
    blue_circles = None
    blue_mask = cv.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    red_mask = cv.bitwise_or(cv.inRange(hsv, RED_LOWER, RED_UPPER), cv.inRange(hsv, RED2_LOWER, RED2_UPPER))
        
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
           
            halfFieldRange = ((width-float(circle[2]))/2)
            xFromCenter = circle[1] - (width/2)
            angle = round((fov/2)  * (xFromCenter/halfFieldRange),4)  

            ball_array.append({
                'id': len(ball_array),
                'color': 'red',
                'centerY': float(circle[1]),
                'centerX': float(circle[0]),
                'radius': float(circle[2]),
                'angle' : angle})
    if blue_circles is not None:
        blue_circles = np.uint16(np.around(blue_circles))
        for circle in blue_circles[0, :]:
            cv.circle(frame, (circle[0], circle[1]), circle[2], (255, 0, 0), 2)

            halfFieldRange = ((width-float(circle[2]))/2)

            xFromCenter = circle[0] - (width/2)
            angle = round((fov/2)  * (xFromCenter/halfFieldRange),4) 

            ball_array.append({
                'id': len(ball_array),
                'color': 'blue',
                'centerY': float(circle[1]),
                'centerX': xFromCenter,
                'radius': float(circle[2]),
                'angle' : angle})
    # Post to network tables
    jsonData = json.dumps(ball_array)
    print(jsonData)
    # TODO: Fix the below code
    sd.putString('jsonData', jsonData)

    # Add desired frames to queue for debug viewing
    returnedFramesQueue.put((frame, red_mask, blue_mask))
    threadsBeingUsed -= 1

detection = threading.Thread(target=detection_thread, name="detection_thread")
detection.start()

fps = 15
prev = 0

if __name__ == '__main__':
    # Open the camera
    cap = cv.VideoCapture(0)
    warningNotDelivered = True
    sink = cs.getVideo()
    while True:
        # print("Currently using", threadsBeingUsed, "threads")
        # Put next available task on thread
        # Lets start with max 3 (even though we can do
        # 4 with Raspberry Pi) just to safety-test
        time_elapsed = time.time() - prev
        if threadsBeingUsed <= 3 and not tasks.empty():
            threadsBeingUsed += 1
            tasks.get().start()

        ret, frame = cvSink.grabFrame(frame)

        # ret, frame = cap.read()
        globalFrame = frame

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
            frame, red_mask, blue_mask = returnedFramesQueue.get()

        # Exit on 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Close the camera
    cap.release()
    cv.destroyAllWindows()










