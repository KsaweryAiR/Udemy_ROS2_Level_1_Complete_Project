#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from functools import partial

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtle_killer_interfaces.msg import Turtle
from turtle_killer_interfaces.msg import TurtleArray
from turtle_killer_interfaces.srv import CatchTurtle


class TurtleControllerNodeV2(Node): 
    def __init__(self):
        super().__init__("turtle_controller_v2") 
        self.get_logger().info("turtle_controller_v2")

        self.declare_parameter("turtle_speed", 3.0)
        self.turtlePose_x_ = 0.0
        self.turtlePose_y_ = 0.0
        self.turtlePose_theta_ = 0.0
        self.turtles_array_ = []
        self.speed_ = self.get_parameter("turtle_speed").value

        self.TurtleArray_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.TurtlePose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle1_pose, 10)
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(1.0/self.speed_, self.publish_move_turtle)
    

    #############################subscriber 

    def callback_alive_turtles(self, msg):
        self.turtles_array_ = msg.turtles
        
    def callback_turtle1_pose(self, msg):
        self.turtlePose_x_ = msg.x
        self.turtlePose_y_ = msg.y
        self.turtlePose_theta_ = msg.theta
    
    #############################function

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calculate_track(self):
        if not self.turtles_array_:  
            self.get_logger().warn("No turtles in the array.")
            return "empty", 0.0, 0.0, float('inf'), float('inf') 
        
        min_street = float('inf')
        name_min_turtle = "empty"
        rotate = 0 
        min_delta_x = 0.0
        min_delta_y = 0.0
        
        for turtle in self.turtles_array_:
            delta_x = turtle.x - self.turtlePose_x_
            delta_y = turtle.y - self.turtlePose_y_
            street  = math.sqrt((delta_x)**2+(delta_y)**2)

            if street < min_street:
                min_street = street
                rotate = math.atan2(delta_y, delta_x) - self.turtlePose_theta_
                rotate = self.normalize_angle(rotate)
                name_min_turtle = turtle.name
                min_delta_x = delta_x
                min_delta_y = delta_y

        return name_min_turtle, min_street, rotate, min_delta_x, min_delta_y
    
    #############################publisher  

    def publish_move_turtle(self):
        twist = Twist()
        name_min_turtle, min_street, rotate, delta_x, delta_y = self.calculate_track()
        if name_min_turtle != "empty":
            if abs(rotate) <= 0.001:
                twist.angular.z = 0.0
                twist.linear.x = min_street * self.speed_
            else:
                twist.angular.z = rotate * self.speed_*1.5
                twist.linear.x = min_street * self.speed_
            
            if abs(delta_x) <= 0.07 and abs(delta_y) <= 0.07:
                self.call_catch_turtle_server(name_min_turtle)
                twist.angular.z = 0.0
            self.publisher_.publish(twist)
        else:
            self.get_logger().warn("EMPTY ARRAY")


    #############################client           

    def call_catch_turtle_server(self, catch_turtle):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server catch_turtle...")
        
        request = CatchTurtle.Request()
        request.catch_turtle = catch_turtle

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle, catch_turtle = catch_turtle))

    def callback_call_catch_turtle(self, futue, catch_turtle):
        try:
            response = futue.result()
            self.get_logger().info(f"{response.succes}")
        except Exception as e:
            self.get_logger().error("Service call failer %r" % (e,))


    


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNodeV2() 
    rclpy.spin(node) 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
