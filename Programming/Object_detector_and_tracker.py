import cv2
import numpy as np
import serial
import time

class ObjectTracker:
    def __init__(self):
        self.center_coordinates = []
        #Load an image
        self.video = cv2.VideoCapture(0)
        ret, frame = self.video.read()

        #Image size
        h, w, c = frame.shape
        print(frame.shape)
        self.height = h
        self.width = w
        self.channels = c
        self.center_coordinates = [h/2, w/2]

        #Initiate CSRT tracker
        self.tracker = cv2.TrackerKCF_create()

        #Boolean value to know if there is an object detected
        self.isObject = False

        #Initialisation of object bounding box
        self.bbox = None

        #Joints limits: FOV = 75°; Motor at rest = 90°
        self.angleMin = 90 - 75/2
        self.angleMax = 90 + 75/2

        # Define the HSV range for green color
        self.lower_range = np.array([35, 100, 100])  # Lower range for green in HSV
        self.upper_range = np.array([85, 255, 255])  # Upper range for green in HSV
        self.hue_range = 20

        #FPS:
        self.frame_count = 0
        self.start_time = 0
        self.fps = 0

        print("Object tracker class initialised")

    def calibration(self, hsv_image):
        # Calibration function for the HSV parameters of the objects to track positioned on the center
        x = int(self.center_coordinates[1])
        y = int(self.center_coordinates[0])
        hue, saturation, value = hsv_image[y][x]

        hue_range = self.hue_range

        hue_lower = hue-hue_range
        if hue_lower < 0:
            hue_lower = hue_lower + 360
        saturation_lower = saturation-50
        if saturation_lower < 0:
            saturation_lower = 0
        value_lower = value-50
        if value_lower < 0:
            value_lower = 0

        self.lower_range = np.array([hue_lower, saturation_lower, value_lower])

        hue_upper = hue+hue_range
        if hue_upper > 360:
            hue_upper = hue_upper - 360
        saturation_upper = saturation+50
        if saturation_upper>255:
            saturation_upper = 255
        value_upper = value+50
        if value_upper > 255:
            value_upper = 255

        self.upper_range = np.array([hue_upper, saturation_upper, value_upper])
    
    def new_frame(self, x_img, y_img):
        # Function to change from image coordinate frame to standard X,Y coordinate frame
        self.newX = x_img
        self.newY = self.height - y_img

    def remap(self, value, min, max, new_min, new_max):
        new_value = int(new_min + (value-min)*(new_max-new_min)/(max-min))
        return new_value

    def get_center_coordinates(self):
        ret, image = self.video.read()

        bbox = self.bbox

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create a mask to isolate green regions
        green_mask = cv2.inRange(hsv_image, self.lower_range, self.upper_range)

        new_edges = np.zeros((self.height, self.width), dtype=np.uint8)

        if bbox is not None:
            x, y, h, w = bbox
            
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to reduce noise
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            # Detect edges using Canny edge detection
            edges = cv2.Canny(blur, 100, 200)

            new_edges[y:y+h, x:x+w] = edges[y:y+h, x:x+w]

        mask = green_mask + new_edges

        # Find contours in the green mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (0, 255, 255))  # draw contours on image

        # Filter contours based on shape (assumes a cube)
        detected_cubes = []
        min_cube_area = areaTmp = 1000
        largest_contour = []

        # Calculate and store the areas of each contour in the dictionary
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)

            if area >= min_cube_area:
                # Approximate the contour to reduce points
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Check if the shape is approximately a cube
                if len(approx) == 4:
                    detected_cubes.append(approx)
                    if area > areaTmp:
                        areaTmp = area
                        largest_contour = contour

        self.isObject = False
        # Draw coordinates of the center of the biggest object
        if len(largest_contour) > 0:
            self.isObject = True
            x, y, w, h = cv2.boundingRect(largest_contour)
            bbox = (x, y, w, h)
            x_center = int(x + w / 2)
            y_center = int(y + h / 2)
            cv2.circle(image, (x_center, y_center), 3, (255, 0, 0), 2)

        if self.isObject and bbox is not None:
            # Update tracker
            self.tracker = cv2.TrackerKCF_create()
            self.tracker.init(image, tuple(bbox))

        if bbox is not None:
            ret, bbox = self.tracker.update(image)
            if ret:
                # Tracking success
                # Draw bounding box
                (x, y, w, h) = [int(v) for v in bbox]
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                Xcenter = x + w/2
                Ycenter = y + h/2
                self.new_frame(Xcenter, Ycenter)
                self.center_coordinates_object = [self.newX, self.newY]
            else:
                bbox = None

        self.bbox = bbox

        # Calculate FPS
        self.frame_count += 1
        cv2.putText(image, f"FPS: {self.fps:.2f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        if self.frame_count % 10 == 0:
            self.end_time = time.time()
            self.fps = self.frame_count / (self.end_time - self.start_time)
            self.start_time = time.time()
            self.frame_count = 0

        # Show the image with rectangles around detected cubes
        cv2.imshow('Detected object', image)
        cv2.imshow('Mask frame', mask)

        if cv2.waitKey(1) & 0xFF == ord('g'):
            self.calibration(hsv_image)

if __name__ == '__main__':

    myTracker = ObjectTracker()
    SerialObj = serial.Serial(port='COM6')
    SerialObj.baudrate = 115200  # set Baud rate to 115200
    SerialObj.bytesize = 8   # Number of data bits = 8
    SerialObj.parity  = 'N'   # No parity
    SerialObj.stopbits = 1   # Number of Stop bits = 1
    time.sleep(1) # Waits for initialisation of USB 
    data = ('<' + str(90) + ',' + str(90) + '>').encode() #Data structure is "<X,Y>"
    SerialObj.write(data)
    
    frame_count = 0
    
    while True:
        myTracker.get_center_coordinates()
        if(myTracker.bbox):
            X = myTracker.center_coordinates_object[0]
            Y = myTracker.center_coordinates_object[1]
            angle_x = myTracker.remap(X, 0, myTracker.width, myTracker.angleMin, myTracker.angleMax)
            angle_y = myTracker.remap(Y, 0, myTracker.height, myTracker.angleMin, myTracker.angleMax)
            print(angle_x,angle_y)
            data = ('<' + str(angle_x) + ',' + str(angle_y) + '>').encode() #Data structure is "<X,Y>"
            SerialObj.write(data)    #transmit X and Y coordinates as a string (byte series) through serial port
            #data = SerialObj.readline() #Read the data received and decoded by the arduino for debugging
            #print(data)
            
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
            break;

    # Release the VideoCapture and close all OpenCV windows
    SerialObj.close()      # Close the COM port
    myTracker.video.release()
    cv2.destroyAllWindows()