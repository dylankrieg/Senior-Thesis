from Ax12 import Ax12
import math
class Motors:
    def __init__(self):
        # e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
        # sets baudrate and opens com port
        Ax12.DEVICENAME = 'COM3'
        Ax12.BAUDRATE = 1_000_000
        # sets baudrate and opens com port
        Ax12.connect()
        # create AX12 instance with ID 1 and 2
        motor_id1 = 1
        motor_id2 = 2
        self.Motor1 = Ax12(motor_id1)
        self.Motor2 = Ax12(motor_id2)
        #speed is in bits from 0-1023 for CCW; 1024 -2047 CW
        self.speed = 100
        self.Motor1.set_moving_speed(self.speed)
        self.Motor2.set_moving_speed(self.speed)
        #the theta that gives parallel position of ID 1
        self.parallel_theta1 = 150
        #the theta that gives parallel position of ID 2
        self.parallel_theta2 = 155
        #servo joint length
        self.servojoint = 84
        # torque in bits from 0-1023
        self.torque = 200
        self.Motor1.set_torque_limit(self.torque)
        self.Motor2.set_torque_limit(self.torque)
        
    #this is before you attach your motors to the gripper       
    def setup(self):
        self.Motor1.set_goal_position(0)
        self.Motor2.set_goal_position(1023)
        
    def position(self, desiredPosition):
        #constraints are set because of grippers kinematic limitations(DO NOT CHANGE)
        constraint1 = self.parallel_theta1 - 150 + 90
        constraint2 = self.parallel_theta1 - 150 + 175
        #need this because the theta is opposite
        difference = self.parallel_theta1 - desiredPosition
        desiredPosition2 = self.parallel_theta2 + difference
        if constraint1 < desiredPosition < constraint2:
            desiredPosition = round(desiredPosition * 3.41)
            desiredPosition2 = round(desiredPosition2 * 3.41)

            self.Motor1.set_goal_position(desiredPosition)
            #other motor needs to be opposite
            self.Motor2.set_goal_position(desiredPosition2)
        else:
            print("invalid position")
            
    def torquelimit(self, torqueLimit):
        self.Motor1.set_torque_limit(torqueLimit)
        self.Motor2.set_torque_limit(torqueLimit)

    def speedlimit(self, speedLimit):
        self.Motor1.set_moving_speed(speedLimit)
        self.Motor2.set_moving_speed(speedLimit)
        
    def theta2distance(self,theta1,theta2):
        #theta needs to be in radians
        #[xpos, ypos to pincher top, ypos to pincher bottom]
        positionFromCenter1 = [math.sin(theta1)*45 + 16.857, math.cos(theta1)*45 - 16.7, math.cos(theta1)*45 - 16.7 + self.servojoint]
        positionFromCenter2 = [math.sin(theta2)*45 - 17.58, math.cos(theta2)*45 - 16.7, math.cos(theta2)*45 - 16.7 + self.servojoint]
        positionFromCenter = [positionFromCenter1, positionFromCenter2]
        return positionFromCenter
    
    def distance2theta(self,xPositionFromCenter):
        x = xPositionFromCenter - 5
        theta1 = math.asin((x - 16.87)/45)
        theta1 = round(150 - math.degrees(theta1))
        #theta comes in degrees
        return theta1
        
    def getEndEffectorPosition(self):
        theta1 = math.radians(150 - (round(self.Motor1.get_present_position() / 3.41)))
        theta2 = math.radians(155 - (round(self.Motor2.get_present_position() / 3.41)))
        positionFromCenter = self.theta2distance(theta1,theta2)
        return positionFromCenter
    
    def openGripper(self):
        self.position(92)
        
    
    def gettheta(self):
        print(round(self.Motor1.get_present_position() / 3.41),"theta1")
        print(round(self.Motor2.get_present_position() / 3.41),"theta2")
        




if __name__ == "__main__":
    #Initializes everything
    Controller = Motors()
    Controller.openGripper()
    """  use this line before attaching motors
    #Controller.setup()   """
    while True:
        print(Controller.getEndEffectorPosition())
        width_object = 70
        half_distance = width_object/2
        print(Controller.distance2theta(half_distance))
        input_pos = int(input("goal pos(91-174 deg): "))
        Controller.position(input_pos)
