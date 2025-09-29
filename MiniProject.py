'''
SEED Lab Assignment 2: Aruco Detection
Zach Brewer
9/15/25
Code captures a video from the usb camera, converts it to greyscale, runs the arucodetection algorithm
then displays it in color with the aruco detection overlayed.
The code also outputs the id of any aruco markers found on the LCD screen, or outputs "no Aruco markers found"
Code should be run on Raspberry Pi Model 4 using Python 3.11.2

'''
from time import sleep
import numpy as np
import cv2
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from cv2 import aruco

lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
camera = cv2.VideoCapture(0)
sleep(.5)

lcdChar = -1;
display  = 0;
while True:

    ret, frame = camera.read()
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
    overlay = frame.copy()

    if not ids is None:
        aruco.drawDetectedMarkers(overlay, corners, ids)
        ids = ids.flatten()
        for (outline, marker_id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
        #print(corners)
        xMid = (corners[0][0][0][0]+ corners[0][0][1][0]+corners[0][0][2][0]+ corners[0][0][3][0])/4
        yMid = (corners[0][0][0][1]+ corners[0][0][1][1]+corners[0][0][2][1]+ corners[0][0][3][1])/4

        #xMid = corners[0][0][0][0]+abs(corners[0][0][0][0] - corners[0][0][1][0])
        #yMid = corners[0][0][0][1]+abs(corners[0][0][0][1] - corners[0][0][2][1])
        xdif = xMid - frame.shape[1]/2
        ydif = yMid - frame.shape[0]/2
        left_wheel = 0 if ydif < 0 else 1
        right_wheel = 1 if xdif < 0 else 0
        #print(left_wheel)
        #print(right_wheel)
        lcdChar = left_wheel + 2*right_wheel        
        
        
    else:
        #cv2.putText(overlay, "No ArUco markers found", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        lcdChar = -1;
        
    cv2.imshow("Aruco Detection", overlay)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if lcdChar != display:
        display = lcdChar
        lcd.clear()
        if display == -1:
            lcd.message = "No Aruco's \nDetected"
        else:
            lcd.message = "Goal Position: \n" + str(left_wheel) + " " + str(right_wheel)
    
lcd.clear()
camera.release()
cv2.destroyAllWindows()
