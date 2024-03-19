#!/usr/bin/env python


# reference: https://gitee.com/liqing415/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
from __future__ import print_function

import threading

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import random
from math import sqrt, fabs, sin, cos, pi

import yaml
import io
import sys, select, termios, tty, time


class __rpy():
    def init(self):
        self.roll = 0.
        self.pitch = 0.
        self.yaw = 0.    
    roll  = 0.
    pitch = 0.
    yaw   = 0.


class __vec3f:
    def init(self):
        self.x = 0.
        self.y = 0.
        self.z = 0.
    
    x = 0
    y = 0
    z = 0

def get_yaml_data(fileDir):
    resList = []

    fo = io.open(fileDir,"r",encoding='utf-8')

    waypts = yaml.load(fo,Loader=yaml.Loader)
  #  print(waypts)

  #  del waypts[0]
  #  for one in waypts:
  #      resList.append((one["data"],one["resp"]))
    return waypts

def data_saturation(x, upper, lower):
    if x > upper:
        return upper
    elif x < lower:
        return lower
    else:
        return x

def is_all_rover_positions_obtained():
    # TODO
    pass

def pos_err(x, y, x_d, y_d, threshold = 0.35):
    # in meters
    dst_err = sqrt((x - x_d) ** 2 + (y - y_d) ** 2)
    if dst_err > threshold:
        return False
    else:
        return True

global current_state, is_uav_state_updated
current_state = State()
is_uav_state_updated = False
def state_cb(msg):
    global current_state, is_uav_state_updated
    current_state = msg

    is_uav_state_updated = True

global flight_pose, is_uav_uwb_position_updated, uwb_signal_loss_time_instant, dt_uwb_loss
flight_pose = PoseStamped()
is_uav_uwb_position_updated = False
uwb_signal_loss_time_instant = 0
dt_uwb_loss = 0
def flight_position_cb(msg_flight_pos):
    global flight_pose, is_uav_uwb_position_updated, uwb_signal_loss_time_instant, dt_uwb_loss
    flight_pose.pose.position.x = msg_flight_pos.pose.position.x
    flight_pose.pose.position.y = msg_flight_pos.pose.position.y
    flight_pose.pose.position.z = msg_flight_pos.pose.position.z

    flight_pose.pose.orientation.x = msg_flight_pos.pose.orientation.x
    flight_pose.pose.orientation.y = msg_flight_pos.pose.orientation.y
    flight_pose.pose.orientation.z = msg_flight_pos.pose.orientation.z
    flight_pose.pose.orientation.w = msg_flight_pos.pose.orientation.w

    is_uav_uwb_position_updated = True

'''
global flight_vel, is_uav_uwb_velocity_updated
flight_vel = TwistStamped()
is_uav_uwb_velocity_updated = False
def flight_velocity_cb(msg_flight_vel):
    global flight_vel, is_uav_uwb_velocity_updated
    flight_vel.twist.linear.x = msg_flight_vel.twist.linear.x
    flight_vel.twist.linear.y = msg_flight_vel.twist.linear.y
    flight_vel.twist.linear.z = msg_flight_vel.twist.linear.z

    is_uav_uwb_velocity_updated = True
'''

global robot_pos_pixel, robot_pos, is_robot_pixel_pos_updated, is_robot_real_pos_updated
robot_pos_pixel = Int32MultiArray(data = [ -1 , -1])
robot_pos = PoseStamped()
is_robot_pixel_pos_updated = False
is_robot_real_pos_updated  = False
def rover_pixel_pos_cb(msg):
    global robot_pos_pixel, is_robot_pixel_pos_updated
    robot_pos_pixel.data[0] = msg.data[0]                   # need to be checked
    robot_pos_pixel.data[1] = msg.data[1]

    is_robot_pixel_pos_updated = True

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed_xy = 0.0
        self.speed_z = 0.0
        self.turn = 0.0
        
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.dst_x = 0
        self.dst_y = 0
        self.dst_z = 0
        self.dst_yaw = 0

        self.up_x = 0
        self.up_y = 0
        self.up_z = 0
        self.uv_x = 0
        self.uv_y = 0
        self.uv_z = 0

        self.uv_yaw = 0
        self.uy_yaw = 0

        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
    
    def update_uv_dst_pos(self, dst_pos, uv, dst_yaw, th):
        self.dst_x = dst_pos.x
        self.dst_y = dst_pos.y
        self.dst_z = dst_pos.z
        self.dst_yaw = dst_yaw

        self.uv_x = uv.x
        self.uv_y = uv.y
        self.uv_z = uv.z

        self.th   = th

    def update_params(self, x, y, z, th, speed_xy, speed_z, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed_xy = speed_xy
        self.speed_z   = speed_z
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update_params(0, 0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = TwistStamped()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            #
            # update Euler angle
            [self.roll, self.pitch, self.yaw] = euler_from_quaternion([flight_pose.pose.orientation.x, flight_pose.pose.orientation.y, flight_pose.pose.orientation.z, flight_pose.pose.orientation.w])

            #
            # P control
            err_x = self.dst_x - flight_pose.pose.position.x
            err_y = self.dst_y - flight_pose.pose.position.y
            err_z = self.dst_z - flight_pose.pose.position.z
            err_yaw = self.dst_yaw - self.yaw

            self.up_x = self.speed_xy * err_x
            self.up_y = self.speed_xy * err_y
            self.up_z = self.speed_z  * err_z
            self.uv_yaw = self.th * self.turn
            self.uy_yaw = turn * err_yaw

            # Copy state into twist message.
            '''
            twist.twist.linear.x  = self.x * self.speed
            twist.twist.linear.y  = self.y * self.speed
            twist.twist.linear.z  = self.z * self.speed
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = self.th * self.turn
            '''
            twist.twist.linear.x  = self.up_x + self.uv_x
            twist.twist.linear.y  = self.up_y + self.uv_y
            twist.twist.linear.z  = self.up_z + self.uv_z
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z =  self.uv_yaw  + self.uy_yaw

            self.condition.release()

            # 
            # coordinate transormation
            # FEEL FREE TO CHANGE!
            twist.twist.linear.x  = self.up_x + self.uv_x
            twist.twist.linear.y  = -self.up_y - self.uv_y
            twist.twist.linear.z  =  self.up_z + self.uv_z            
            

            #
            # data saturation
            # FEEL FREE TO CHANGE!
            twist.twist.linear.x = data_saturation(twist.twist.linear.x, 0.5,  -0.5)
            twist.twist.linear.y = data_saturation(twist.twist.linear.y, 0.5,  -0.5)
            twist.twist.linear.z = data_saturation(twist.twist.linear.z, 0.25, -0.25)

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.twist.linear.x = 0
        twist.twist.linear.y = 0
        twist.twist.linear.z = 0
        twist.twist.angular.x = 0
        twist.twist.angular.y = 0
        twist.twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def target_pos_vel(dst_pos, current_pos, up, uv, uw_yaw, uy_yaw):
    return "[Control Node] \tX_d: (%s, %s, %s)\tX: (%s, %s, %s)\n\t\tu_pos: (%s, %s, %s)\tu_vel: (%s, %s, %s)\tu_yaw: (%s, %s)  \n" % (dst_pos.x, dst_pos.y, dst_pos.z, flight_pose.pose.position.x, flight_pose.pose.position.y, flight_pose.pose.position.z, up.x, up.y, up.z, uv.x, uv.y, uv.z, uw_yaw, uy_yaw)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    #
    # initialize ROS node
    rospy.init_node('uav_nuc_keyboard_ctrl')

    #
    # load ROS params
    speed_xy = rospy.get_param("~speed_xy", 0.2)
    speed_xy_tracking = rospy.get_param("~speed_xy_tracking", 0.2)
    speed_z  = rospy.get_param("~speed_z",  0.2)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 20.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    searching_alt = rospy.get_param("~tracking_alt", 2.5)
    tracking_alt  = rospy.get_param("~tracking_alt", 1.5)

    
    #
    # load camera params
    .cam_x_pixel = rospy.get_param("~cam_x_pixel", 320)
    cam_y_pixel = rospy.get_param("~cam_y_pixel", 240)
    cam_f       = rospy.get_param("~cam_f", 1.6)

    pub_thread = PublishThread(repeat)

    #
    # initialize UAV
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    
    #rospy.wait_for_service("/mavros/cmd/arming")                                           # disabled for debugging
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    #rospy.wait_for_service("/mavros/set_mode")                                             # disabled for debugging
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    #
    # initialize UWB
    flight_pos_sub = rospy.Subscriber("/uav_2/uwb_pose/pose", PoseStamped, callback = flight_position_cb)
    #flight_vel_sub = rospy.Subscriber("/uav_2/uwb_pose/vel", TwistStamped, callback = flight_velocity_cb)

    #
    # initialize Camera
    rover_pixel_pos_sub = rospy.Subscriber("mobile_robot_from_camera_1", Int32MultiArray, callback = rover_pixel_pos_cb)           # need to be checked
    rover_real_pos_pub  = rospy.Publisher('/rover_real_pos/', PoseStamped, queue_size = 1)

    #
    # initialize waypoints
    waypts = get_yaml_data('/home/ubuntu484/catkin_ws_ros/src/uav_planning/scripts/test_rover/data.yaml')                              # need to be checked

    #
    # Setpoint publishing MUST be faster than 2Hz
    # Wait for Flight Controller connection
    rate = rospy.Rate(20)
    for i in range(0, 60):
        rate.sleep()

    #
    # arming UAV
    '''
    is_armed = False                                                                        # disabled for debugging
    last_req_arm = rospy.Time.now()    
    while not rospy.is_shutdown() and not is_armed:
        if rospy.Time.now() - last_req_arm > rospy.Duration(1.0):
            #is_armed = arming_client.call(arm_cmd).success
            arming_client.call(arm_cmd)
            is_armed = True

            rospy.loginfo("Arming vehicle...")                       #print(type(arming_client.call(arm_cmd)))

            last_req_arm = rospy.Time.now()
        
        rate.sleep()
    '''
    rospy.loginfo("Vehicle armed!")
    
	
    time_startup = rospy.get_time()
    last_time = rospy.get_time()

    #
    # get home position
    home_pos = __vec3f()
    '''
    while not is_uav_uwb_position_updated:
        rate.sleep()                                                                                                    # disabled for debugging
    if is_uav_uwb_position_updated:
        home_pos.x = flight_pose.pose.position.x
        home_pos.y = flight_pose.pose.position.y
        home_pos.z = 1.0                                                                                        # flight_pose.pose.position.z

        is_uav_uwb_position_updated = False
    #else:                                                                                                                    # for debugging, take off first
    #    home_pos.z = 1.0
    '''
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    dst_pos = __vec3f()
    dst_yaw        = 0.

    up = __vec3f()
    uv = __vec3f()
    uw_yaw = 0
    uy_yaw = 0

    try:
        #pub_thread.wait_for_subscribers()                                                   # disabled for debugging
        pub_thread.update_params(x, y, z, th, speed_xy, speed_z, turn)

        #
        # set initial position
        dst_pos.x  =  home_pos.x
        dst_pos.y  =  home_pos.y
        dst_pos.z  =  home_pos.z

        err_z = dst_pos.z - flight_pose.pose.position.z                              # give the inital order or the uav will disarm
        up.z = speed_z  * err_z                                                             # need to be checked, takeoff will not be stopped
        print(str( dst_pos.z) + "  " + str(flight_pose.pose.position.z))
        pub_thread.update_uv_dst_pos(dst_pos, uv, dst_yaw, th)

        #print(msg)
        #print(vels(speed_xy,turn))
        print(target_pos_vel(dst_pos, flight_pose, up, uv, uw_yaw, uy_yaw))
        while(True):

            #
            # if the uwb signal is successfully received
            #if is_uav_uwb_position_updated:                                                 # disabled for debugging
            if True:

                #
                # Pre-processing
                is_robot_real_pos_updated = False
                x_t = 0
                y_t = 0

                rpy = __rpy()
                [rpy.roll, rpy.pitch, rpy.yaw] = euler_from_quaternion([flight_pose.pose.orientation.x, flight_pose.pose.orientation.y, flight_pose.pose.orientation.z, flight_pose.pose.orientation.w])


                #if is_robot_pixel_pos_updated:
                if True:                                                                    # disabled for debugging
                
                    # pixel pos -> real pos, horizontal frame
                    x_pixel = robot_pos_pixel.data[0] - cam_x_pixel / 2                     # need to be checked
                    y_pixel = robot_pos_pixel.data[1] - cam_y_pixel / 2                     # xy position in pixel, origin centered

                    x_t = x_pixel / cam_f * flight_pose.pose.position.z
                    y_t = x_pixel / cam_f * flight_pose.pose.position.z

                    x_t = x_t / cos(rpy.roll)                                               # need to be checked
                    y_t = y_t / cos(rpy.pitch)

                    # horizontal frame -> global frame (ENU)
                    robot_pos.pose.position.x = x_t *  cos(-(pi / 2 - rpy.yaw)) + y_t * sin(-(pi / 2 - rpy.yaw))
                    robot_pos.pose.position.y = x_t * -sin(-(pi / 2 - rpy.yaw)) + y_t * cos(-(pi / 2 - rpy.yaw))

                    robot_pos.pose.position.x += flight_pose.pose.position.x
                    robot_pos.pose.position.y += flight_pose.pose.position.y

                    # publish rover real position to ground station
                    robot_pos.header.stamp = rospy.Time.now()
                    rover_real_pos_pub.publish(robot_pos)

                    is_robot_pixel_pos_updated = False
                    is_robot_real_pos_updated  = True


                #
                # TODO
                # COLLISION AVOIDANCE by overriding control
                # Command is sent by ground station
                #if is_pos_vel_ctrl_override:
                if False:                                                                       # disabled
                    uv.x  = 0
                    uv.y  = 0
                    uv.z  = 0
                    th = 0

                    dst_pos.x  =  flight_pose.pose.position.x
                    dst_pos.y  =  flight_pose.pose.position.y
                    dst_pos.z  =  flight_pose.pose.position.z
                    th     = 0
                    uw_yaw = 0

                    pub_thread.update_uv_dst_pos(dst_pos, uv, dst_yaw, th)
                    continue

                # if the target rover is not found
                # which can be MODIFIED
                #   if the positions of all 3 rovers are not obtained: 
                #if not is_robot_pixel_pos_updated:                                             # disabled for debugging
                #if not is_all_rover_positions_obtained():
                if True:
                    
                    waypt_numbers = waypts.__len__()

                    for i in range(1, waypt_numbers):
                        t_start_waypt = waypts["point"+str(i)]["t1"]
                        t_end_waypt   = waypts["point"+str(i)]["t2"] 

                        if rospy.get_time() - time_startup >= t_start_waypt and rospy.get_time() - time_startup < t_end_waypt:
                            
                            now_time = rospy.get_time() 
                            d_t= now_time - last_time

                            dst_pos.x =  waypts["point"+str(i)]["x"]
                            dst_pos.y =  waypts["point"+str(i)]["y"]
                            dst_pos.z =  searching_alt

                            if (status == 14):
                                rospy.loginfo("[Control Node] DETECTING... " + str(rospy.get_time() - time_startup) + "\t Waypt:" + "point"+str(i) + " " + str(waypts["point"+str(i)]["x"]) + ", " + str(waypts["point"+str(i)]["y"]))

                else:
                    # move to rover position
                    if not pos_err(flight_pose.pose.position.x, flight_pose.pose.position.y, robot_pos.pose.position.x, robot_pos.pose.position.y):
                            dst_pos.x = robot_pos.pose.position.x
                            dst_pos.y = robot_pos.pose.position.y
                            dst_pos.z = tracking_alt
                    else:
                        if is_robot_real_pos_updated:
                            # track using local frame
                            # velocity P-control, FEEL FREE to change the control law!
                            uv.x  = robot_pos.pose.position.x - flight_pose.pose.position.x
                            uv.y  = robot_pos.pose.position.x - flight_pose.pose.position.x
                            uv.z  = 0
                            th = 0

                            uv.x = speed_xy_tracking * uv.x
                            uv.y = speed_xy_tracking * uv.y

                            dst_pos.x  =  flight_pose.pose.position.x
                            dst_pos.y  =  flight_pose.pose.position.y
                            dst_pos.z  =  flight_pose.pose.position.z

                            is_robot_real_pos_updated = False

                        else:
                            # if tracking is loss
                            # then gain altitude
                            uv.x  = 0
                            uv.y  = 0
                            uv.z  = 0
                            th = 0

                            dst_pos.x  =  flight_pose.pose.position.x
                            dst_pos.y  =  flight_pose.pose.position.y
                            dst_pos.z  =  4.5
         
                is_uav_uwb_position_updated = False
                dt_uwb_loss = 0
            else:

                if dt_uwb_loss == 0:
                    uwb_signal_loss_time_instant = rospy.get_time()

                dt_uwb_loss = rospy.get_time() - time_startup


                if dt_uwb_loss >= 4:
                    # assume the UWB signal is blocked, then MOVE
                    uv.x  =  random.uniform(-0.2, 0.2)
                    uv.y  = random.uniform(-0.2, 0.2)
                    uv.z  = 0
                    th    = 0

                    dst_pos.x  =  flight_pose.pose.position.x
                    dst_pos.y  =  flight_pose.pose.position.y
                    dst_pos.z  =  flight_pose.pose.position.z
                    th     = 0
                    uw_yaw = 0
                #
                # if the signal is loss over 1 second
                elif dt_uwb_loss >= 1:
                    uv.x  = 0
                    uv.y  = 0
                    uv.z  = 0
                    th = 0

                    dst_pos.x  =  flight_pose.pose.position.x
                    dst_pos.y  =  flight_pose.pose.position.y
                    dst_pos.z  =  flight_pose.pose.position.z
                    th     = 0
                    uw_yaw = 0

                    if (status == 14):
                        rospy.loginfo("[Control Node] UWB signal LOSS ...!")


            if (status == 14):
                #print(vels(speed_xy,turn))
                print(target_pos_vel(dst_pos, flight_pose, up, uv, uw_yaw, uy_yaw))
                rospy.loginfo("[Control Node] UAV Position: " + str(flight_pose.pose.position.x) + "\t" + str(flight_pose.pose.position.y) + "\t" + str(flight_pose.pose.position.z))
                pass
            status = (status + 1) % 15

            #pub_thread.update_params(x, y, z, th, speed_xy, speed_z, turn)
            pub_thread.update_uv_dst_pos(dst_pos, uv, dst_yaw, th)

            key = getKey(0.025)
            if key == '\x03' or key == 'q':
                break
            #time.sleep(0.025)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

