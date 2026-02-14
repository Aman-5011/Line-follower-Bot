import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import smbus
import random

prev_time = time.time()

# GPIO Pin Configuration
IN1 = 25
IN2 = 26
IN3 = 30
IN4 = 31
PWM1 = 32
PWM2 = 33

#GPIO_TRIGGER =45
#GPIO_ECHO =9

IR_PINS = [17, 27, 22, 5, 6]

list_path =[]
wtg = 0

# PID Constants
KP = 8 # Proportional Gain
KD = 2 # Derivative Gain
TURN_SPEED = 30
BASE_SPEED = 50

# Initialize MPU6050
mpu = mpu6050(0x68)
current_angle = 0


# GPIO Setup
for pin IR_PINS:
    GPIO.setup(pin ,GPIO.IN)

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
#GPIO.setup(GPIO_TRIGGER , GPIO.OUT)
#GPIO.setup(GPIO_ECHO , GPIO.IN)
GPIO.setup(PWM1, GPIO.OUT)
GPIO.setup(PWM2, GPIO.OUT)
left_pwm = GPIO.PWM(PWM1, 1500)
right_pwm = GPIO.PWM(PWM2, 1500)
left_pwm.start(0)
right_pwm.start(0)
last_error=0, error=0

# Functions to control movement
def update_angle():
    global current_angle, prev_time
    
    now = time.time()  # Get current time in seconds
    dt = now - prev_time  # Compute time difference
    prev_time = now  # Update previous time
    
    # Read gyroscope data
    gyro_data = mpu.get_gyro_data()
    gyroZ = gyro_data['z']  # Get Z-axis rotation rate in degrees per second

    # Update the angle using integration
    current_angle += gyroZ * dt

    return current_angle

def turn_theta(theta):
    global current_angle
    current_angle = 0
    turn_speed = 80  # Initial turn speed

    if theta > 0:  # Clockwise turn (right)
        GPIO.output(IN1, GPIO.HIGH)  
        GPIO.output(IN2, GPIO.LOW)  
        GPIO.output(IN3, GPIO.LOW)  
        GPIO.output(IN4, GPIO.HIGH) 
    else:  # Counterclockwise turn (left)
        GPIO.output(IN1, GPIO.LOW)   # Left motor backward
        GPIO.output(IN2, GPIO.HIGH)  # Right motor forward
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

    while abs(current_angle) < abs(theta):
        left_pwm.ChangeDutyCycle(turn_speed)
        right_pwm.ChangeDutyCycle(turn_speed)
        
        update_angle()  # Update gyro reading
        time.sleep(0.005)

    stop_motors()



def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    left_pwm.ChangeDutyCycle(BASE_SPEED)
    right_pwm.ChangeDutyCycle(BASE_SPEED)

def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    left_pwm.ChangeDutyCycle(BASE_SPEED - 10)
    right_pwm.ChangeDutyCycle(BASE_SPEED + 10)

def turn_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    left_pwm.ChangeDutyCycle(BASE_SPEED + 10)
    right_pwm.ChangeDutyCycle(BASE_SPEED - 10)

def sharp_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    left_pwm.ChangeDutyCycle(BASE_SPEED - 30)
    right_pwm.ChangeDutyCycle(BASE_SPEED + 30)

def sharp_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    left_pwm.ChangeDutyCycle(BASE_SPEED + 30)
    right_pwm.ChangeDutyCycle(BASE_SPEED - 30)

def stop_motors():
    left_pwm.ChangeDutyCycle(0)  # Ensure PWM stops sending power
    right_pwm.ChangeDutyCycle(0)
    
    GPIO.output(IN1, GPIO.LOW)  # Stop motor driver signals
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(0.1)  # Small delay for stability

def read_ir_sensors():
    """ Read the values of the 6 IR sensors and return an array. """
    return [GPIO.input(pin) for pin in IR_PINS]

def move_forward1():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    left_pwm.ChangeDutyCycle(BASE_SPEED)
    right_pwm.ChangeDutyCycle(BASE_SPEED)

    for i in range(3):
    matrix_binary = extract_matrix()
    if matrix_binary:
        decimal_value = convert_matrix_to_decimal(matrix_binary)
        if i==0:
            control_speed(decimal_value)
        elif i==1:
            reset_speed()
        else:
         #GYRO LOGIC
                    pass #yahn par gyro wala code daal dena 

def follow_line():
    global path_end
    ir_values = read_ir_sensors()
    
    if (GPIO.input(IR_PINS[0]) == 1 and GPIO.input(IR_PINS[1]) == 1 and GPIO.input(IR_PINS[2]) == 1 and GPIO.input(IR_PINS[3]) == 1 and GPIO.input(IR_PINS[4]) == 1 ):
        path_end=True
        print("no path further")
        L=len(list_path)
        list_path.remove(list[L-1][0])
        turn_theta(180)
        wtg=(wtg+2)%4
        #nodes.remove(options[])
    
    run_pid()

    if GPIO.input(IR_PINS[2]) == 0 :
        move_forward1()
        
    elif GPIO.input(IR_PINS[1]) == 0:
        turn_left()
        
    elif GPIO.input(IR_PINS[3]) == 0:
        turn_right()
        
    elif GPIO.input(IR_PINS[0]) == 0:
        sharp_left()
        
    elif GPIO.input(IR_PINS[4]) == 0:
        sharp_right()

    #check for intersection
    sensor_sum=0
    for i in range(6):
        sensor_sum += ( GPIO.input(i))
        if sensor_sum == 0:
            move_forward(.2)
            if( GPIO.input(IR_PINS[2]==0)):
                stop_motors()
                get_options("P")
                
            else:
                stop_motors()
                get_options("T")
                
        elif (GPIO.input(IR_PINS) == [1, 1, 0, 0, 0, 0] ):
            move_forward(.2)
            if GPIO.input(IR_PINS[2]==0) :
                stop_motors()
                get_options("SR")
                

        elif(GPIO.input(IR_PINS) == [0, 0, 0, 0, 1, 1]):
            move_forward(.2)
            if GPIO.input(IR_PINS[2]==0):
                stop_motors()
                get_options("SL")
                

    return None

def get_options(char):
    if path_end :
        path_end=False
        
    
    elif back_track :
        back_track = False
        
    else:
        if char == 'P':
            node =['S','L','R','B']

        elif char == 'T':
            node = [ 'L','R','B']

        elif char =='SR':
            node = ['S','R','B']

        else :
            node = [ 'S','L','B']
        
        list_path.append(node)
    
           
    try_path(list_path)   
    return node


        
def try_path(list_path):
    L=len(list_path)
    for i in range(L-1,-1,-1):
        if (list[i][0] == 'S'):
            move_forward(0.1)
            follow_line()
            return 

        elif (list[i][0] == 'L'):
            wtg = (wtg+1) % 4
            if wtg == 0:
                move_forward(0.4)
                follow_line()
            if wtg ==1 :
                turn_theta(-90)
                
                move_forward(0.4)
                follow_line
           
            if wtg ==3 :
                turn_theta(90)
                
                move_forward(0.4)
                follow_line

        elif (list[i][0] == 'R'):
            wtg = (wtg-1) % 4
            if wtg == 0:
                move_forward(0.2)
                follow_line()
            if wtg ==1 :
                turn_theta(-90)
                move_forward(0.2)
                follow_line
           
            if wtg ==3 :
                turn_theta(90)
                
                move_forward(0.4)
                follow_line

        elif (list[i][0] == 'B'):
            global back_track 
            back_track=True
            wtg = (wtg+2) % 4
            L=len(list)
            list.remove(list[L-1][0])
            L=len(list)
            list.remove(list[L-1][0])
            if wtg == 0:
                wtg=2
                move_forward(0.4)
                follow_line()
            if wtg ==1 :
                wtg=2
                turn_theta(-90)
                move_forward(0.4)
                follow_line
           
            if wtg ==3 :
                wtg=2
                turn_theta(90)
                move_forward(0.4)
                follow_line()
    return None

def extract_matrix():

    # Open the camera
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FPS, 15)

    # Convert the whole frame to grayscale
    #gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Store the last time the binary value was calculated
    last_millis = time.time()
    delay_ms = .2  # 1s delay

    ret, frame = cap.read()
    if not ret:
        print("Error: Could not capture frame")
        cap.release()
        return None
            
    # Convert the whole frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Get frame dimensions
    height, width, _ = frame.shape

    # Define the number of grid rows and columns
    rows, cols = 3, 4  

    # Compute step size for grid lines
    step_x = width // (cols + 1)
    step_y = height // (rows + 1)

    # Draw vertical grid lines
    for i in range(1, cols + 1):
        x = i * step_x
        cv2.line(frame, (x, 0), (x, height), (0, 255, 0), 2)

    # Draw horizontal grid lines
    for j in range(1, rows + 1):
        y = j * step_y
        cv2.line(frame, (0, y), (width, y), (0, 255, 0), 2)

    # Define main bounding box coordinates
    x1, y1, x2, y2 = 128, 120, 384, 360  

    # Draw main bounding box
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    #ACCORDING TO AMAN1........

    # Define square inner bounding boxes inside the main box
    square_size = (x2 - x1) // 4  # Make the squares 1/4 the width of the main box

    # First square (upper center)
    sq1_x1 = x1 + ((x2 - x1) // 2) - square_size 
    sq1_y1 = y1  
    sq1_x2 = x1 + ((x2 - x1) // 2) + square_size 
    sq1_y2 = y1 + ((y2 - y1) // 2) 
        
    # Second square (lower center)
    sq2_x1 = sq1_x1
    sq2_y1 = y1 + ((y2 - y1) // 2) 
    sq2_x2 = x1 + ((x2 - x1) // 2) + square_size 
    sq2_y2 = y1 + (y2 - y1)

    # Draw the inner squares
    cv2.rectangle(frame, (sq1_x1, sq1_y1), (sq1_x2, sq1_y2), (255, 255, 255), 2)  # Top square
    cv2.rectangle(frame, (sq2_x1, sq2_y1), (sq2_x2, sq2_y2), (0, 255, 255), 2)  # Bottom square


    #ACCORDING TO AMAN2..................

    # Define square inner bounding boxes inside the main box
    square_size = (x2 - x1) // 4   # Make the squares 1/4 the width of the main box

    # First square (left vertical box)
    sqL_x1 = x1 
    sqL_y1 = y1  
    sqL_x2 = x1 + ((x2 - x1) // 2) - square_size 
    sqL_y2 = y2 

    # Second square (right vertical box)
    sqR_x1 = x1 + ((x2 - x1) // 2) + square_size 
    sqR_y1 = y1
    sqR_x2 = x2
    sqR_y2 = y2

    # Draw the inner squares
    cv2.rectangle(frame, (sqL_x1, sqL_y1), (sqL_x2, sqL_y2), (0, 0, 0), 2)  # Top square
    cv2.rectangle(frame, (sqR_x1, sqR_y1), (sqR_x2, sqR_y2), (0, 0, 255), 2)  # Bottom square


    #ACCORDING TO AMAN3...............

    # Define the size of the new bounding boxes
    outer_box_height = (y2 - y1) // 4  # 1/4 of main bounding box height
    outer_box_width = x2 - x1  # Same width as main bounding box

    square_size = (x2 - x1) // 4
    # Define upper bounding box (centered above main box)
    upper_x1 = x1 + ((x2 - x1) // 2) - square_size 
    upper_y1 = 0
    upper_x2 = x1 + ((x2 - x1) // 2) + square_size
    upper_y2 = y1

    # Define lower bounding box (centered below main box)
    lower_x1 = x1 + ((x2 - x1) // 2) - square_size
    lower_y1 = y2  # Just below the main box
    lower_x2 = x1 + ((x2 - x1) // 2) + square_size
    lower_y2 =480

    # Draw the new outer bounding boxes
    cv2.rectangle(frame, (upper_x1, upper_y1), (upper_x2, upper_y2), (255, 255, 0), 2)  # Upper Box (Yellow)
    cv2.rectangle(frame, (lower_x1, lower_y1), (lower_x2, lower_y2), (255, 255, 255), 2)  # Lower Box (Purple)

    #BOX_1.................

    B1_x1 = x1
    B1_y1 = 0
    B1_x2 = x1 + ((x2 - x1) // 2) - square_size 
    B1_y2 = y1

    cv2.rectangle(frame, (B1_x1, B1_y1), (B1_x2, B1_y2), (50, 100, 200), 3)  # Upper Box (Yellow)
            

    #BOX_2.................
    B2_x1 = x1 + ((x2 - x1) // 2) + square_size 
    B2_y1 = 0
    B2_x2 = x2
    B2_y2 = y1

    cv2.rectangle(frame, (B2_x1, B2_y1), (B2_x2, B2_y2), (200, 100, 0), 3)  # Upper Box (Yellow)
            

    #BOX_3.................
    B3_x1 = x1
    B3_y1 = y2
    B3_x2 = x1 + ((x2 - x1) // 2) - square_size 
    B3_y2 = 480

    cv2.rectangle(frame, (B3_x1, B3_y1), (B3_x2, B3_y2), (200, 50, 10), 3)  # Upper Box (Yellow)
            

    #BOX_4.................
    B4_x1 = x1 + ((x2 - x1) // 2) + square_size 
    B4_y1 = y2
    B4_x2 = x2
    B4_y2 = 480

    cv2.rectangle(frame, (B4_x1, B4_y1), (B4_x2, B4_y2), (200, 0, 100), 3)  # Upper Box (Yellow)
            

    # Get the current time
    current_millis = time.time()

    # Check if 1s has passed
    if (current_millis - last_millis) >= delay_ms:
        last_millis = current_millis  # Reset timer

        # Convert the whole frame to grayscale
        #gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #WHITE RATIO FOR AMAN1.........................

        # Extract the inner boxes
        inner_box1 = gray_frame[sq1_y1:sq1_y2, sq1_x1:sq1_x2]
        inner_box2 = gray_frame[sq2_y1:sq2_y2, sq2_x1:sq2_x2]

        # Convert to simple thresholding (global threshold)
        _, binary_inner1 = cv2.threshold(inner_box1, 128, 255, cv2.THRESH_BINARY)
        _, binary_inner2 = cv2.threshold(inner_box2, 128, 255, cv2.THRESH_BINARY)


        # Calculate the white pixel ratio in both small boxes
        white_ratio1 = np.sum(binary_inner1 == 255) / binary_inner1.size
        white_ratio2 = np.sum(binary_inner2 == 255) / binary_inner2.size

        #cv2.imshow("Binary Inner 1", binary_inner1)
        #cv2.imshow("Binary Inner 2", binary_inner2)

        #print(f"White Ratio 1: {white_ratio1:.2f}, White Ratio 2: {white_ratio2:.2f}")

        #WHITE RATIO FOR AMAN2................................

        # Extract the inner boxes
        left_box = gray_frame[sqL_y1:sqL_y2, sqL_x1:sqL_x2]
        right_box = gray_frame[sqR_y1:sqR_y2, sqR_x1:sqR_x2]

        # Convert to simple thresholding (global threshold)
        _, binary_LEFT = cv2.threshold(left_box, 128, 255, cv2.THRESH_BINARY)
        _, binary_RIGHT = cv2.threshold(right_box, 128, 255, cv2.THRESH_BINARY)


        # Calculate the white pixel ratio in both small boxes
        white_ratio_LEFT = np.sum(binary_LEFT == 255) / binary_LEFT.size
        white_ratio_RIGHT = np.sum(binary_RIGHT == 255) / binary_RIGHT.size

        #cv2.imshow("Binary LEFT", binary_LEFT)
        #cv2.imshow("Binary RIGHT", binary_RIGHT)

        #print(f"White Ratio LEFT: {white_ratio_LEFT:.2f}, White Ratio RIGHT: {white_ratio_RIGHT:.2f}")

                
        #WHITE RATIO FOR AMAN3...........................

        # Extract the new outer bounding boxes
        upper_box = gray_frame[upper_y1:upper_y2, upper_x1:upper_x2]
        lower_box = gray_frame[lower_y1:lower_y2, lower_x1:lower_x2]
                
        _, binary_upper = cv2.threshold(upper_box, 128, 255, cv2.THRESH_BINARY)
        _, binary_lower = cv2.threshold(lower_box, 128, 255, cv2.THRESH_BINARY)
                
        white_ratio_upper = np.sum(binary_upper == 255) / binary_upper.size
        white_ratio_lower = np.sum(binary_lower == 255) / binary_lower.size

        #print(f"White Ratio Upper: {white_ratio_upper:.2f}, White Ratio Lower: {white_ratio_lower:.2f}")
                


        #WHITE RATIO FOR BOX 1,2,3,4 ................................

        # Extract the new outer bounding boxes
        B1 = gray_frame[B1_y1:B1_y2, B1_x1:B1_x2]
        B2 = gray_frame[B2_y1:B2_y2, B2_x1:B2_x2]
        B3 = gray_frame[B3_y1:B3_y2, B3_x1:B3_x2]
        B4 = gray_frame[B4_y1:B4_y2, B4_x1:B4_x2]
                
        _, binary_B1 = cv2.threshold(B1, 128, 255, cv2.THRESH_BINARY)
        _, binary_B2 = cv2.threshold(B2, 128, 255, cv2.THRESH_BINARY)
        _, binary_B3 = cv2.threshold(B3, 128, 255, cv2.THRESH_BINARY)
        _, binary_B4 = cv2.threshold(B4, 128, 255, cv2.THRESH_BINARY)
                
        white_ratio_B1 = np.sum(binary_B1 == 255) / binary_B1.size
        white_ratio_B2 = np.sum(binary_B2 == 255) / binary_B2.size
        white_ratio_B3 = np.sum(binary_B3 == 255) / binary_B3.size
        white_ratio_B4 = np.sum(binary_B4 == 255) / binary_B4.size

        print(f"White Ratio BOX_1: {white_ratio_B1:.2f}, White Ratio BOX_2: {white_ratio_B2:.2f}")
        print(f"White Ratio BOX_3: {white_ratio_B3:.2f}, White Ratio BOX_4: {white_ratio_B4:.2f}")



        if white_ratio_upper>0.25 or white_ratio_lower>0.25:
            print("skip1")
        else:
            if white_ratio1>0.7 and white_ratio2>0.7:
                A_val = 0
                B_val = 0
                C_val = 0
                D_val = 0

                # Print Binary Value
                binary_value = f"{A_val}{B_val}{C_val}{D_val}"
                print(f"Binary Value (ABCD): {binary_value}")

            else:
                # If both small boxes have more than 70% black pixels, skip processing the big box
                if white_ratio_B1<0.7 and white_ratio_B2<0.7 and white_ratio_B3<0.7 and white_ratio_B4<0.7 : 
                    print("skip2")
                else:
                    if white_ratio_LEFT <0.55 or white_ratio_RIGHT <0.55:
                        # Process the big bounding box
                        roi = gray_frame[y1:y2, x1:x2]

                        # Apply threshold (black = 1, white = 0)
                        _, binary = cv2.threshold(roi, 128, 255, cv2.THRESH_BINARY_INV)

                        # Divide into 4 quadrants
                        h, w = binary.shape
                        half_h, half_w = h // 2, w // 2

                        A = binary[0:half_h, 0:half_w]  # Top-left
                        B = binary[0:half_h, half_w:w]  # Top-right
                        C = binary[half_h:h, 0:half_w]  # Bottom-left
                        D = binary[half_h:h, half_w:w]  # Bottom-right

                        # Function to get binary value based on pixel intensity
                        def get_binary_value(region):
                            return 1 if np.mean(region) > 127 else 0  

                        # Get binary values
                        #A_val = get_binary_value(A)
                        #B_val = get_binary_value(B)
                        #C_val = get_binary_value(C)
                        #D_val = get_binary_value(D)
                        for i in range(3)
                            if i==0 :
                                A_val= 1
                                B_val= 1
                                C_val= 1
                                D_val= 0
                            if i==1 :
                                A_val= 0
                                B_val= 0
                                C_val= 0
                                D_val= 0
                            if i==2 :
                                A_val= 1
                                B_val= 0
                                C_val= 1
                                D_val= 0

                        # Print Binary Value
                        binary_value = f"{A_val}{B_val}{C_val}{D_val}"
                        print(f"Binary Value (ABCD): {binary_value}")
                        return binary_value

                                    
                    else:
                        print("skip3")
                            


        # Display the frame
        cv2.imshow("Live Stream", frame)
        #cv2.imshow("gray", gray_frame) 

        # Press 'q' to exit
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break
    # Release the camera and close windows
    cap.release()
    cv2.destroyAllWindows()
    return None

def convert_matrix_to_decimal(binary_str):
    return int(binary_str,2)

def control_speed(value):
    #bot can move only at speed of 0.6541 m/s
    if value<=3    
        max_speed = value / 5.0
        left_pwm.ChangeDutyCycle(max_speed * 100)
        right_pwm.ChangeDutyCycle(max_speed * 100)
    elif value>3 and value>0:
        max_speed = value / 5.0
        MAX_SPEED_VALUE = 15  # Maximum value from matrix
        MAX_DUTY_CYCLE = 100  # PWM range 0-100%
        speed_value = max(0, min(speed_value, MAX_SPEED_VALUE))  # Scale speed value to duty cycle
        duty_cycle = (speed_value / MAX_SPEED_VALUE) * MAX_DUTY_CYCLE  
        left_pwm.ChangeDutyCycle(min(100, max(0, duty_cycle)))
        right_pwm.ChangeDutyCycle(min(100, max(0, duty_cycle)))



try:
    while True:
        follow_line()
except KeyboardInterrupt:
    print("\nProgram interrupted by user!") 

finally: 
    print("Stopping Motors & Cleaning up GPIO...")
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()  # Release GPIO resources
GPIO.cleanup()