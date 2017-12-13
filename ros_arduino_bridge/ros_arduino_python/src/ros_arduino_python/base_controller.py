#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os

from math import sin, cos, pi, sqrt
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32

""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame, name="base_controllers"):
        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 20))
        self.timeout = rospy.get_param("~base_controller_timeout", 0.3)
        self.stopped = False
        self.debugPID = False

        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "")
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "")
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        
        pid_params['AWheel_Kp'] = rospy.get_param("~AWheel_Kp", 11)
        pid_params['AWheel_Kd'] = rospy.get_param("~AWheel_Kd", 15)
        pid_params['AWheel_Ki'] = rospy.get_param("~AWheel_Ki", 0)
        pid_params['AWheel_Ko'] = rospy.get_param("~AWheel_Ko", 50)

        pid_params['BWheel_Kp'] = rospy.get_param("~BWheel_Kp", 11)
        pid_params['BWheel_Kd'] = rospy.get_param("~BWheel_Kd", 15)
        pid_params['BWheel_Ki'] = rospy.get_param("~BWheel_Ki", 0)
        pid_params['BWheel_Ko'] = rospy.get_param("~BWheel_Ko", 50)

        pid_params['CWheel_Kp'] = rospy.get_param("~CWheel_Kp", 11)
        pid_params['CWheel_Kd'] = rospy.get_param("~CWheel_Kd", 16)
        pid_params['CWheel_Ki'] = rospy.get_param("~CWheel_Ki", 0)
        pid_params['CWheel_Ko'] = rospy.get_param("~CWheel_Ko", 50)
        
        pid_params['DWheel_Kp'] = rospy.get_param("~DWheel_Kp", 11)
        pid_params['DWheel_Kd'] = rospy.get_param("~DWheel_Kd", 16)
        pid_params['DWheel_Ki'] = rospy.get_param("~DWheel_Ki", 0)
        pid_params['DWheel_Ko'] = rospy.get_param("~DWheel_Ko", 50)

        self.accel_limit = rospy.get_param('~accel_limit', 0.05)
        self.debugPID = rospy.get_param('~debugPID', False)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        self.linear_scale_correction = rospy.get_param("~linear_scale_correction", 1.0)
        self.angular_scale_correction = rospy.get_param("~angular_scale_correction", 1.0)

        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)

        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction / (self.wheel_diameter * pi)
        self.ticks_per_meter = self.ticks_per_meter * self.linear_scale_correction
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        now = rospy.Time.now()
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data
        self.enc_A = None            # encoder readings
        self.enc_B = None
        self.enc_C = None
        self.enc_D = None

        self.x  = 0                  # position in xy plane
        self.y  = 0
        self.th = 0                  # rotation in radians

        self.v_A = 0
        self.v_B = 0
        self.v_C = 0
        self.v_D = 0

        self.v_des_AWheel = 0        # cmd_vel setpoint
        self.v_des_BWheel = 0
        self.v_des_CWheel = 0
        self.v_des_DWheel = 0

        self.last_cmd_vel = now

        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

        # Clear any old odometry info
        self.arduino.reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()

        # rqt_plot debug pid
        if self.debugPID:
            self.AEncoderPub = rospy.Publisher('Aencoder', Int32, queue_size=10)
            self.BEncoderPub = rospy.Publisher('Bencoder', Int32, queue_size=10)
            self.CEncoderPub = rospy.Publisher('Cencoder', Int32, queue_size=10)
            self.DEncoderPub = rospy.Publisher('Dencoder', Int32, queue_size=10)
            
            self.APidoutPub  = rospy.Publisher('Apidout',  Int32, queue_size=10)
            self.BPidoutPub  = rospy.Publisher('Bpidout',  Int32, queue_size=10)
            self.CPidoutPub  = rospy.Publisher('Cpidout',  Int32, queue_size=10)
            self.DPidoutPub  = rospy.Publisher('Dpidout',  Int32, queue_size=10)
            
            self.AVelPub     = rospy.Publisher('Avel',     Int32, queue_size=10)
            self.BVelPub     = rospy.Publisher('Bvel',     Int32, queue_size=10)
            self.CVelPub     = rospy.Publisher('Cvel',     Int32, queue_size=10)
            self.DVelPub     = rospy.Publisher('Dvel',     Int32, queue_size=10)

        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True

        if missing_params:
            os._exit(1)

        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.wheel_track = self.wheel_track * self.angular_scale_correction
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']

        self.AWheel_Kp = pid_params['AWheel_Kp']
        self.AWheel_Kd = pid_params['AWheel_Kd']
        self.AWheel_Ki = pid_params['AWheel_Ki']
        self.AWheel_Ko = pid_params['AWheel_Ko']

        self.BWheel_Kp = pid_params['BWheel_Kp']
        self.BWheel_Kd = pid_params['BWheel_Kd']
        self.BWheel_Ki = pid_params['BWheel_Ki']
        self.BWheel_Ko = pid_params['BWheel_Ko']

        self.CWheel_Kp = pid_params['CWheel_Kp']
        self.CWheel_Kd = pid_params['CWheel_Kd']
        self.CWheel_Ki = pid_params['CWheel_Ki']
        self.CWheel_Ko = pid_params['CWheel_Ko']
        
        self.DWheel_Kp = pid_params['DWheel_Kp']
        self.DWheel_Kd = pid_params['DWheel_Kd']
        self.DWheel_Ki = pid_params['DWheel_Ki']
        self.DWheel_Ko = pid_params['DWheel_Ko']

        self.arduino.update_pid(self.AWheel_Kp, self.AWheel_Kd, self.AWheel_Ki, self.AWheel_Ko,
                                self.BWheel_Kp, self.BWheel_Kd, self.BWheel_Ki, self.BWheel_Ko,
                                self.CWheel_Kp, self.CWheel_Kd, self.CWheel_Ki, self.CWheel_Ko,self.DWheel_Kp, self.DWheel_Kd, self.DWheel_Ki, self.DWheel_Ko)

    def poll(self):
        if self.debugPID:
            try:
                A_pidin, B_pidin, C_pidin, D_pidin = self.arduino.get_pidin()
                self.AEncoderPub.publish(A_pidin)
                self.BEncoderPub.publish(B_pidin)
                self.CEncoderPub.publish(C_pidin)
                self.DEncoderPub.publish(D_pidin)
            except:
                rospy.logerr("getpidin exception count:")
                return

            try:
                A_pidout, B_pidout, C_pidout, D_pidout = self.arduino.get_pidout()
                self.APidoutPub.publish(A_pidout)
                self.BPidoutPub.publish(B_pidout)
                self.CPidoutPub.publish(C_pidout)
                self.DPidoutPub.publish(D_pidout)
            except:
                rospy.logerr("getpidout exception count")
                return

        now = rospy.Time.now()
        if now > self.t_next:
            # Read the encoders
            try:
                aWheel_enc, bWheel_enc, cWheel_enc, dWheel_enc = self.arduino.get_encoder_counts()
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
            #rospy.loginfo("Encoder A:"+str(aWheel_enc)+",B:"+str(bWheel_enc)+",C:" + str(cWheel_enc))

            dt = now - self.then
            self.then = now
            dt = dt.to_sec()

            # Calculate odometry
            if self.enc_A == None and self.enc_B == None and self.enc_C == None and self.enc_D == None:
                d_A = 0
                d_B = 0
                d_C = 0
                d_D = 0
            else:
                d_A = (aWheel_enc - self.enc_A) / self.ticks_per_meter
                d_B = (bWheel_enc - self.enc_B) / self.ticks_per_meter
                d_C = (cWheel_enc - self.enc_C) / self.ticks_per_meter
                d_D = (dWheel_enc - self.enc_D) / self.ticks_per_meter

            self.enc_A = aWheel_enc
            self.enc_B = bWheel_enc
            self.enc_C = cWheel_enc
            self.enc_D = dWheel_enc

            va = d_A/dt;
            vb = d_B/dt;
            vc = d_C/dt;
            vd = d_D/dt;
            
			# forward kinematic

            vx = (va + vb + vc + vd) / 4
            vy = (-va + vb - vc + vd) / 4
            vth = (-va - vb + vc + vd) / (4 * self.wheel_track)

            delta_x = (vx*cos(self.th) - vy*sin(self.th))*dt
            delta_y = (vx*sin(self.th) + vy*cos(self.th))*dt
            delta_th = vth*dt;

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)

            #Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                "odom"
                )

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
           # odom.pose.covariance = [1e-9, 0, 0, 0, 0, 0,
            #                        0, 1e-3, 1e-9, 0, 0, 0,
             #                       0, 0, 1e6, 0, 0, 0,
              #                      0, 0, 0, 1e6, 0, 0,
               #                     0, 0, 0, 0, 1e6, 0,
                #                    0, 0, 0, 0, 0, 1e-9]

            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth
           # odom.twist.covariance = [1e-9, 0, 0, 0, 0, 0,
            #                        0, 1e-3, 1e-9, 0, 0, 0,
             #                       0, 0, 1e6, 0, 0, 0,
              #                      0, 0, 0, 1e6, 0, 0,
               #                     0, 0, 0, 0, 1e6, 0,
                #                    0, 0, 0, 0, 0, 1e-9]

            self.odomPub.publish(odom)

            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_AWheel = 0
                self.v_des_BWheel = 0
                self.v_des_CWheel = 0
                self.v_des_DWheel = 0

            if self.v_A < self.v_des_AWheel:
                self.v_A += self.max_accel
                if self.v_A > self.v_des_AWheel:
                    self.v_A = self.v_des_AWheel
            else:
                self.v_A -= self.max_accel
                if self.v_A < self.v_des_AWheel:
                    self.v_A = self.v_des_AWheel

            if self.v_B < self.v_des_BWheel:
                self.v_B += self.max_accel
                if self.v_B > self.v_des_BWheel:
                    self.v_B = self.v_des_BWheel
            else:
                self.v_B -= self.max_accel
                if self.v_B < self.v_des_BWheel:
                    self.v_B = self.v_des_BWheel

            if self.v_C < self.v_des_CWheel:
                self.v_C += self.max_accel
                if self.v_C > self.v_des_CWheel:
                    self.v_C = self.v_des_CWheel
            else:
                self.v_C -= self.max_accel
                if self.v_C < self.v_des_CWheel:
                    self.v_C = self.v_des_CWheel
                    
            if self.v_D < self.v_des_DWheel:
                self.v_D += self.max_accel
                if self.v_D > self.v_des_DWheel:
                    self.v_D = self.v_des_DWheel
            else:
                self.v_D -= self.max_accel
                if self.v_D < self.v_des_DWheel:
                    self.v_D = self.v_des_DWheel        

            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_A, self.v_B, self.v_C, self.v_D)
                if self.debugPID:
                    self.AVelPub.publish(self.v_A)
                    self.BVelPub.publish(self.v_B)
                    self.CVelPub.publish(self.v_C)
                    self.CVelPub.publish(self.v_D)

            self.t_next = now + self.t_delta

    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0, 0, 0)

    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()

        v_x  = req.linear.x      # m/s
        v_y  = req.linear.y      # m/s
        v_th = req.angular.z     # rad/s

        #reverse kinematic
        vA = v_x - v_y - 2.6 * v_th
        vB = v_x + v_y - 2.6 * v_th
        vC = v_x - v_y + 2.6 * v_th
        vD = v_x + v_y + 2.6 * v_th
        
              
        self.v_des_AWheel = int(vA * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_BWheel = int(vB * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_CWheel = int(vC * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_DWheel = int(vD * self.ticks_per_meter / self.arduino.PID_RATE)
        #rospy.loginfo("A v_des:"+str(self.v_des_AWheel)+",B v_des:"+str(self.v_des_BWheel)+",C v_des:" + str(self.v_des_CWheel))


