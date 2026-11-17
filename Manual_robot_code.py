from cyberpi import *
import gamepad
import math
while True:
    
    val1 = gamepad.get_joystick('Lx')
    val2 = gamepad.get_joystick('Ly')
    val3 = gamepad.get_joystick('Rx')
    val4 = gamepad.get_joystick('Ry')
    position = math.sqrt(pow(val1,2)+pow(val2,2))
    
    serv_angle = (180/100)*val4
    motor_spd = (50/100)*val2
    rotate = (70/100)*val3
    
    ## Tip servo
    if gamepad.is_key_pressed('R1'):
        mbot2.servo_set(180,'s3')
    else:
        mbot2.servo_set(45,'s3')
          
    ## Digital Servo
    # if val4 != 0:
    #     mbot2.servo_set(serv_angle,'s4')
    if gamepad.is_key_pressed('N2'):
        mbot2.servo_set(150,'s4')
    else:
        mbot2.servo_set(30,'s4')
        
    ## Encoder
    if val2 != 0 and val2 <= 100 and val2 >= -100:
        mbot2.drive_power(-motor_spd,motor_spd)
    elif val1 != 0 and val1 > 0 and val1 <= 100:
        mbot2.drive_power(-val1,0)
    elif val1 != 0 and val1 < 0 and val1 >= -100:
        mbot2.drive_power(0,-val1)
    elif val3 != 0 and val2 <= 100 and val2 >= -100:
        mbot2.drive_power(rotate,rotate)
    else:
        mbot2.EM_stop()

        
    ## DC motor       
    if gamepad.is_key_pressed('R1'):
        mbot2.motor_set(100,'M2')
    else:
        mbot2.motor_set(0,'M2')
    if gamepad.is_key_pressed('L1'):
        mbot2.motor_set(-150,'M2')
    else:
        mbot2.motor_set(0,'M2')