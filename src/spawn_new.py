#!/usr/bin/python

import rospy
import math
import random
import geometry_msgs.msg
from turtlesim.srv import Spawn
from mas_framework.msg import base
from mas_framework.msg import status
from mas_framework.msg import trade
import turtlesim.msg as turtle


class Turtle:

    def __init__(self, number, angular, linear, max_distance, goal_id):

        self.number = number
        self.name = 'turtle'+str(self.number)
        self.status = "wait"
	self.max_distance = max_distance
        self.linear_velocity = linear
        self.angular_velocity = angular
        self.general_goal_name = 'turtle'+str(goal_id)
        self.local_goal_name = self.general_goal_name
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
	self.price = 0.0
        self.turtles = {self.name: [self.x, self.y, self.phi]}
	self.turtles_prices = {self.name: self.price}
        self.borders_points = {"A": [0, 10], "B": [0, 0], "C": [10, 0], "D": [10, 10]}
        self.base_msg = base()
	self.trade_msg = trade()
        self.geometry_msg = geometry_msgs.msg.Twist()
        self.velocity_publisher = rospy.Publisher(self.name + "/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.coordinate_publisher = rospy.Publisher("/environment", base, queue_size=1)
	self.trade_publisher = rospy.Publisher("/trade", trade, queue_size=1)
        self.action_publisher = rospy.Publisher(self.name + "/action", base, queue_size=1)
        self.pose_subscriber = rospy.Subscriber(self.name + '/pose', turtle.Pose, self.CallbackPose, queue_size=1)
        self.coordinate_subscriber = rospy.Subscriber("/environment", base, self.CallbackCoordinate, queue_size=10)
        self.status_msg_subscriber = rospy.Subscriber("/status", status, self.CallbackStatus, queue_size=1)
	self.trade_subscriber = rospy.Subscriber("/trade", trade, self.CallbackTrade, queue_size=10)

    def Start(self):
        # waiting for a msg from the turtle leader
        while self.general_goal_name not in self.turtles:
            rospy.sleep(1)

        rospy.sleep(1)
 
        if self.name != self.general_goal_name:
            self.Trade()

        rospy.sleep(1)

	self.status = "move"

        while not rospy.is_shutdown():
	    
	    if self.name != self.general_goal_name:

                if self.status == "move":

                    if self.DetectBorder():
                        self.Border()

                    self.Move()

    def Move(self):

        x_goal = self.turtles[self.local_goal_name][0]
        y_goal = self.turtles[self.local_goal_name][1]

        phi = self.phi
        theta = self.GetAngle(y_goal - self.y , x_goal- self.x)
        distance = self.GetDistance(x_goal, y_goal, self.x, self.y)

        if distance > self.max_distance:

            angular, linear = self.GetVelocity(phi, theta)
	    self.geometry_msg.angular.z = angular
            self.geometry_msg.linear.x = linear

        else:

            self.geometry_msg.angular.z = 0
            self.geometry_msg.linear.x = 0

        self.velocity_publisher.publish(self.geometry_msg)

    def DetectBorder(self):
        # ABCD is a square, AB is first side, BC is second and etc
        A = self.borders_points["A"]
        B = self.borders_points["B"]
        C = self.borders_points["C"]
        D = self.borders_points["D"]

        dis = self.GetDistanceFromLineAndPoint(A, B, [self.x, self.y])
        if dis < 1:
            return True

        dis = self.GetDistanceFromLineAndPoint(A, D, [self.x, self.y])
        if dis < 1:
            return True

        dis = self.GetDistanceFromLineAndPoint(B, C, [self.x, self.y])
        if dis < 1:
            return True

        dis = self.GetDistanceFromLineAndPoint(C, D, [self.x, self.y])
        if dis < 1:
            return True

    def Border(self):
        
        while self.DetectBorder():

            phi = self.phi
            A = self.borders_points["A"]
            C = self.borders_points["C"]
            # find the center point of ABCD square
            y_goal = 0.5*(A[0]+C[0])
            x_goal = 0.5*(A[1]+C[1])

            theta = self.GetAngle(y_goal - self.y , x_goal- self.x)
            angular, linear = self.GetVelocity(phi, theta)
            self.geometry_msg.angular.z = 2*angular
            self.geometry_msg.linear.x = linear

            self.velocity_publisher.publish(self.geometry_msg)

    def Trade(self):

        price = self.GetPrice()
        self.trade_msg.price = price
        self.trade_msg.name = self.name
        pub = False
        # publish while dont get msg from all turtles
        while len(self.turtles_prices) < len(self.turtles) - 1 or pub == False:
            self.trade_publisher.publish(self.trade_msg)
            pub = True

	self.local_goal_name = self.GetGoalName()

    def SortByPriceAndName(self, element):

        return element[1], element[0]

    def GetPrice(self):
        # price is turtle distance form the goal   
        x_goal = self.turtles[self.general_goal_name][0]
        y_goal = self.turtles[self.general_goal_name][1]
        x = self.x
        y = self.y
        return self.GetDistance(x_goal, y_goal, self.x, self.y)
            

    def GetGoalName(self):
        # convert dictionary (self.turtles_prices) to list of tuples (X from turtleX name, price)
        prices = [(int(''.join(filter(str.isdigit, name))), price) for name, price in self.turtles_prices.items()]
        prices.sort(key=self.SortByPriceAndName)
        # find a number of the turtle
        i = 0
        while i < len(prices):
            if prices[i][0] == self.number:
                number = i
                break
            i+=1
        # find a goal of the turtle and return it
	if number == 0:
            goalname = self.general_goal_name
        else:
            goalname = 'turtle' + str(prices[number - 1][0])

        return goalname

    def GetDistanceFromLineAndPoint(self, LP_1, LP_2, P):
        
        line_length = self.GetDistance(LP_1[0], LP_1[1], LP_2[0], LP_2[1])
        distance = abs((LP_2[1]-LP_1[1])*P[0]-(LP_2[0]-LP_1[0])*P[1]+LP_2[0]*LP_1[1]-LP_2[1]*LP_1[0])/line_length
        return distance

    def GetDistance(self, x1, y1, x2, y2):

        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def GetAngle(self, y, x):

        theta = math.atan2(y, x)

        if theta < 0:
            theta += 2*math.pi

        return theta

    def GetVelocity(self, phi, theta):
        
        if phi < 0:
            phi += 2*math.pi

        delta = phi - theta

        linear = self.linear_velocity
        angular = self.angular_velocity

        if abs(delta) > 0.1 and abs(delta) < 6.18:

            if delta > 0:

                if delta > math.pi:
                    return angular, linear
                else:
                    return -angular, linear
            else:

                if delta > -math.pi:
                    return angular, linear
                else:
                    return -angular, linear
        else:
            return 0, linear

    def CallbackPose(self, data):

        self.x = data.x
        self.y = data.y
        self.phi = data.theta
        self.base_msg.x = data.x
        self.base_msg.y = data.y
        self.base_msg.phi = data.theta
        self.base_msg.load = self.name
        self.coordinate_publisher.publish(self.base_msg)

    def CallbackCoordinate(self, data):

        self.turtles[data.load] = [data.x, data.y, data.phi]

    def CallbackStatus(self, data):

        if self.name != self.general_goal_name and self.general_goal_name in self.turtles:
                self.trade()

        self.status = data.status
        rospy.sleep(1)

    def CallbackTrade(self, data):

        self.turtles_prices[data.name] = [data.price]


if __name__ == "__main__":

    rospy.init_node("~turtle", anonymous=True)
    number = rospy.get_param("~number")
    goal_id = rospy.get_param("/goal_id")
    angular = rospy.get_param("/angular_vel")
    linear = rospy.get_param("/linear_vel")
    max_distance = rospy.get_param("/max_distance")

    rospy.wait_for_service('spawn')

    if number != 1:

        x = rospy.get_param("~x_coordinate")
        y = rospy.get_param("~y_coordinate")
        theta = rospy.get_param("~theta_coordinate")
        spawn_turtle_x = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle_x(x, y, theta, '')

    x = Turtle(number, angular, linear, max_distance, goal_id)
    x.Start()
    rospy.spin()
