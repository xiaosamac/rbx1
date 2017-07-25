#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState, RegionOfInterest, CameraInfo
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64
from math import radians
import thread
import random

class HeadTracker():
    def __init__(self):
        rospy.init_node("head_tracker")
        
        rospy.on_shutdown(self.shutdown)
        
        rate = rospy.get_param("~rate", 50)
        r = rospy.Rate(rate)
        tick = 1.0 / rate
        
        # Keep the speed updates below about 5 Hz; otherwise the servos
        # can behave erratically.
        speed_update_rate = rospy.get_param("~speed_update_rate", 10)
        speed_update_interval = 1.0 / speed_update_rate
        
        # How big a change do we need in speed before we push an update
        # to the servos?
        self.speed_update_threshold = rospy.get_param("~speed_update_threshold", 0.01)
        
        # What are the names of the pan and tilt joints in the list of dynamixels?
        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')
        self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_tilt_joint')
        
        self.joints = [self.head_pan_joint, self.head_tilt_joint]
        
        # Joint speeds are given in radians per second
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 1.0)
        self.max_joint_speed = rospy.get_param('~max_joint_speed', 2.0)

        # How far ahead or behind the target (in radians) should we aim for?
        self.lead_target_angle = rospy.get_param('~lead_target_angle', 1.0)
        
        # The pan/tilt thresholds indicate what percentage of the image window
        # the ROI needs to be off-center before we make a movement
        self.pan_threshold = rospy.get_param("~pan_threshold", 0.05)
        self.tilt_threshold = rospy.get_param("~tilt_threshold", 0.05)
        
        # The gain_pan and gain_tilt parameter determine how responsive the
        # servo movements are. If these are set too high, oscillation can result.
        self.gain_pan = rospy.get_param("~gain_pan", 1.0)
        self.gain_tilt = rospy.get_param("~gain_tilt", 1.0)
        
        # Set limits on the pan and tilt angles
        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param("~min_pan", radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))
        
        # How long we are willing to wait (in seconds) for a target before re-centering the servos?
        self.recenter_timeout = rospy.get_param('~recenter_timeout', 5)
        
        # Monitor the joint states of the pan and tilt servos
        self.joint_state = JointState()
        rospy.Subscriber('joint_states', JointState, self.update_joint_state, queue_size=1)
        
        # Wait until we actually have joint state values
        while self.joint_state == JointState():
            rospy.sleep(1)
        
        # Initialize the servo services and publishers
        self.init_servos()
                
        # Center the pan and tilt servos at the start
        self.center_head_servos()

        # Set a flag to indicate when the target has been lost
        self.target_visible = False
        
        # Set a timer to determine how long a target is no longer visible
        target_lost_timer = 0.0
        
        # Set a timer to track when we do a speed update
        speed_update_timer = 0.0
        
        # Initialize the pan and tilt speeds to zero
        pan_speed = tilt_speed = 0.0
        
        # Get a lock for updating the self.move_cmd values
        #self.lock = thread.allocate_lock()
        
        rospy.wait_for_message('joint_states', JointState)

        
        rospy.loginfo("Ready to track target.")
                
        while not rospy.is_shutdown():
            
            self.pan_speed=1.0
            self.tilt_speed=1.0
 

            random_pan=random.randint(-145, 145) 
            random_pan=radians(random_pan)
            self.pan_position = random_pan;

            random_tilt=random.randint(-90, 90) 
            random_tilt=radians(random_tilt) 
            self.tilt_position = random_tilt

            self.set_servo_speed(self.head_pan_joint, self.pan_speed)
            self.set_servo_speed(self.head_tilt_joint, self.tilt_speed)
                    
                    # Update the pan position   
            if self.last_pan_position != self.pan_position:
               self.set_servo_position(self.head_pan_joint, self.pan_position)
               self.last_pan_position = self.pan_position
                
                    # Update the tilt position
            if self.last_tilt_position != self.tilt_position:
               self.set_servo_position(self.head_tilt_joint, self.tilt_position)
               self.last_tilt_position = self.tilt_position  
            rospy.sleep(2.0)  
            #r.sleep()

                
           
            
    def center_head_servos(self):
        rospy.loginfo("Centering servos.")

        self.servo_speed[self.head_pan_joint](self.default_joint_speed)
        self.servo_speed[self.head_tilt_joint](self.default_joint_speed)
        
        current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        while abs(current_tilt) > 0.05 or abs(current_pan) > 0.05:
            self.servo_position[self.head_pan_joint].publish(0)
            self.servo_position[self.head_tilt_joint].publish(0)
            
            rospy.sleep(0.1)
            
            current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        self.servo_speed[self.head_pan_joint](0.0)
        self.servo_speed[self.head_tilt_joint](0.0)
        
    def init_servos(self):
        # Create dictionaries to hold the speed, position and torque controllers
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()

        # Connect to the set_speed services and define a position publisher for each servo
        rospy.loginfo("Waiting for joint controllers services...")
                
        for joint in sorted(self.joints):
            # The set_speed services
            set_speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(set_speed_service)
            self.servo_speed[joint] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)

            # Initialize the servo speed to the default_joint_speed
            self.servo_speed[joint](self.default_joint_speed)

            # The position controllers
            self.servo_position[joint] = rospy.Publisher('/' + joint + '/command', Float64, queue_size=5)
            
            # A service to enable/disable servo torque
            torque_enable = '/' + joint + '/torque_enable'
            rospy.wait_for_service(torque_enable) 
            self.torque_enable[joint] = rospy.ServiceProxy(torque_enable, TorqueEnable)
            self.torque_enable[joint](False)
        
        self.pan_position = 0
        self.tilt_position = 0
        self.pan_speed = 0
        self.tilt_speed = 0
        
        self.last_pan_position = 0
        self.last_tilt_position = 0
        self.last_tilt_speed = 0
        self.last_pan_speed = 0
        
    def set_servo_speed(self, servo, speed):
        # Guard against a speed of exactly zero which means 
        # "move as fast as you can" to a Dynamixel servo.
        if speed == 0:
            speed = 0.01
        self.servo_speed[servo](speed)
        
    def set_servo_position(self, servo, position):
        self.servo_position[servo].publish(position)
            
    def update_joint_state(self, msg):
        self.joint_state = msg
        
    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        
    def shutdown(self):
        rospy.loginfo("Shutting down head tracking node.")
        
        # Turn off updates from the /roi subscriber
        try:
            self.roi_subscriber.unregister()
        except:
            pass
        
        # Center the servos
        self.center_head_servos()
        
        rospy.sleep(2)
        
        # Relax all servos to give them a rest.
        rospy.loginfo("Relaxing pan and tilt servos.")
        
        for servo in self.joints:
            self.torque_enable[servo](False)
             
                   
if __name__ == '__main__':
    try:
        HeadTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node terminated.")
