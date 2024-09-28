#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from functools import partial

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from turtle_killer_interfaces.msg import Turtle
from turtle_killer_interfaces.msg import TurtleArray
from turtle_killer_interfaces.srv import CatchTurtle


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner") 
        self.get_logger().info("Hello turtle_spawner")

        self.declare_parameter("time_spawner", 1.0)
        self.time_spawner_ = self.get_parameter("time_spawner").value
        self.number_ = 1
        self.group_turtles_ = TurtleArray()
        self.publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.server_= self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle) 
        self.timer_ = self.create_timer(self.time_spawner_, self.publish_turtles)
        self.catch_turtle_ = "empty"

    #############################server
    
    def callback_catch_turtle(self, request, response):
        self.call_turtle_kill_server(request.catch_turtle)
        self.remove_turtle(request.catch_turtle)
        response.success = True
        return response
        
    #############################function

    def location_spawner(self):
        x = round(random.uniform(0.5, 10.5), 3)
        y = round(random.uniform(1, 10), 3)
        theta = round(random.uniform(0.00, 6.28), 3)
        name = f"free_turtle{self.number_}"
        self.number_+=1
        return x, y, theta, name
    
    def remove_turtle(self, death_turtle):
        self.group_turtles_.turtles  = [turtle for turtle in self.group_turtles_.turtles if turtle.name != death_turtle]
    

    #############################publisher 

    def publish_turtles(self):
        new_turtle = Turtle()

        x, y, theta, name = self.location_spawner()
        new_turtle.x = x
        new_turtle.y = y
        new_turtle.theta = theta
        new_turtle.name = name

        self.group_turtles_.turtles.append(new_turtle)

        self.call_turtle_spawn_server(x, y, theta, name)
        self.publisher_.publish(self.group_turtles_)
        

    #############################client 


    def call_turtle_spawn_server(self, x, y, theta, name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Spawn turtle...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn, x=x, y=y, theta=theta, name=name))

    def callback_call_spawn(self, future, x, y, theta, name):
        try:
            response = future.result()
            self.get_logger().info(f"{response.name} has spawned")
        except Exception as e:
            self.get_logger().error("Service call failer %r" % (e,))

    #############################client2
    

    def call_turtle_kill_server(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Kill turtle...")
        
        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill, name=name))

    def callback_call_kill(self, future, name):
        try:
            self.get_logger().info(f"{name} was killed")
        except Exception as e:
            self.get_logger().error("Service call failer %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node) 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    