from cyberpi import *
import gamepad
import math
import time
while True:
    
    val1 = gamepad.get_joystick('Lx')
    val2 = gamepad.get_joystick('Ly')
    val3 = gamepad.get_joystick('Rx')
    val4 = gamepad.get_joystick('Ry')
    position = math.sqrt(pow(val1,2)+pow(val2,2))
    
    serv_angle = (180/100)*val4
    
    ## Tip servo
    if gamepad.is_key_pressed('N1'):
        mbot2.servo_set(180,'s3')
    else:
        mbot2.servo_set(45,'s3')
        
        
    ## Digital Servo
    if val4 <= 100 and val4 >= -100:
        mbot2.servo_set(serv_angle,'s4')
        
    ## Encoder
    if val2 != 0 and val2 <= 100 and val2 >= -100:
        mbot2.drive_power(-val2+30,val2-30)
    elif val1 != 0 and val1 > 0 and val1 <= 100:
        mbot2.drive_power(-val1,0)
    elif val1 != 0 and val1 < 0 and val1 >= -100:
        mbot2.drive_power(0,-val1)
    elif val3 != 0 and val2 <= 100 and val2 >= -100:
        mbot2.drive_power(val3,val3)
    else:
        mbot2.EM_stop()

        
    ## DC motor       
    if gamepad.is_key_pressed('R1'):
        mbot2.motor_set(255,'M2')
        time.sleep(1);
    else:
        mbot2.motor_set(0,'M2')
    if gamepad.is_key_pressed('L1'):
        mbot2.motor_set(-255,'M2')
        time.sleep(1);
    else:
        mbot2.motor_set(0,'M2')