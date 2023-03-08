from Ax12 import Ax12

# e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
Ax12.DEVICENAME = 'COM6'

Ax12.BAUDRATE = 1_000_000

# sets baudrate and opens com port
Ax12.connect()

# create AX12 instance with ID 1 and 2
# speed is in terms of bits from 0- 1023 CCW; 1024-2047 CW
motor_id = 2
my_dxl = Ax12(motor_id)
motor_id2 = 1
my_dxl2 = Ax12(motor_id2)  
my_dxl.set_moving_speed(100)
my_dxl2.set_moving_speed(100)


def user_input():
    """Check to see if user wants to continue"""
    ans = input('Continue? : y/n ')
    if ans == 'n':
        return False
    else:
        return True


def main(motor_object, motor_object2):
    """ sets goal position based on user input """
    bool_test = True
    while bool_test:
        
        print("\nPosition of dxl ID: %d is %d " %
              (motor_object.id, motor_object.get_present_position()))
        print("\nPosition of dxl ID: %d is %d " %
              (motor_object2.id, motor_object2.get_present_position()))
        print("\nTorque Limit of dxl ID: %d is %d " %
              (motor_object2.id, motor_object2.get_torque_limit()))
        # desired angle input
        input_pos = int(input("goal pos(0-1023: "))
        #the other motor has to go the other way
        input_pos2 = abs(input_pos - 1023)
        if input_pos < 1024:
            motor_object.set_goal_position(input_pos) 
            motor_object2.set_goal_position(input_pos2)
        else:
            print("invalid position"%(motor_object.id))
            print("Position of dxl ID: %d is now: %d " %
            (motor_object.id, motor_object.get_present_position()))
            print("Position of dxl ID: %d is now: %d " %
            (motor_object2.id, motor_object2.get_present_position()))
        bool_test = user_input()

# pass in AX12 object
main(my_dxl, my_dxl2)


# disconnect
my_dxl.set_torque_enable(0)
my_dxl2.set_torque_enable(0)
Ax12.disconnect()
