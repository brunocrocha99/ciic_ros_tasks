import rclpy, threading, time, sys, os
from random import randint
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from typing import cast

from launch import LaunchDescription
from launch import LaunchService
import launch_ros.actions

from launch.actions import (EmitEvent, RegisterEventHandler, LogInfo)
from launch.event_handlers import (OnProcessExit, OnProcessIO, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (LocalSubstitution)

from ciic_ros_tasks_messages.msg import TaskBooking

#change path in order to laucnh performer files
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

class TaskBooker(Node):

    def __init__(self):
        super().__init__('task_booker')

        #booker id parameter, defined at init
        self.declare_parameter('my_id', 90) # default value
        self.id = self.get_parameter('my_id').value
        
        #avoids the booking of multiple tasks at the same time
        self.booking = False

        #publisher definition
        self.publisher_ = self.create_publisher(TaskBooking, '/tasks_booking', 10)

        #subscriber definition
        self.get_logger().info('[INIT] STARTED LISTENING ON TOPIC \'tasks_booking\'')
        self.subscription = self.create_subscription(
            TaskBooking,
            '/tasks_booking',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning

    #publish B to book task
    def send_booking_request(self, advertiser_id):
        msg = TaskBooking()
        msg.sender_id = self.id
        msg.recipient_id = advertiser_id
        msg.task_namespace = self.task_namespace
        msg.action = 'B'
        self.publisher_.publish(msg)
        self.get_logger().info('[BOOKING REQUEST] | SENDER_ID:"%d" | RECIPIENT_ID:"%d" | TASK_NAME:"%s" | ACTION:"%s"' % (msg.sender_id, msg.recipient_id, msg.task_namespace, msg.action))

    #process received messages through /tasks_booking topic
    def listener_callback(self, msg):
        if (msg.action == 'B'):
            return

        self.get_logger().info('[RECEIVED TOPIC MSG] SENDER_ID:"%d" | RECIPIENT_ID:"%d" | NAMESPACE:"%s" | ACTION:"%s"' % (msg.sender_id, msg.recipient_id, msg.task_namespace, msg.action))

        if(msg.action == 'A' and not self.booking):
            self.task_namespace = msg.task_namespace
            self.booking = True
            self.send_booking_request(msg.sender_id)            
            return

        if(msg.action == 'S' and msg.recipient_id == self.id):
            self.get_logger().info('[BOOKING APROVED] LAUNCHING \'%s\' NODE' % msg.performer_identifier)
            self.perform_launch(msg.performer_identifier, self.task_namespace)

            return

        if(msg.action == 'F' and msg.recipient_id == self.id):
            self.reset_booking()
            return
    
    #used when task is shutdown
    def reset_booking(self):
        self.booking = False
        self.task_namespace = ''

    #performer node launcher for the booked tasks
    #place the performer files in the same directory as this file
    def perform_launch(self, performer_identifier, task_namespace):
        print('\n\n--------------------------------------------------------\n\n')

        perfomer_node = launch_ros.actions.Node(package = 'ciic_ros_tasks', executable=performer_identifier, output='screen', namespace=task_namespace)

        ld = LaunchDescription([
            perfomer_node,
            RegisterEventHandler(
                OnProcessIO(
                    target_action=perfomer_node,
                    on_stdout=lambda event: LogInfo(
                        msg=format(event.text.decode().strip())
                    )
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=perfomer_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(
                            reason='Task Performer shutdown'))
                    ]
                )
            ),            
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[LogInfo(
                        msg=['Task performer was asked to shutdown: ',
                            LocalSubstitution('event.reason')],
                    ),
                    self.reset_booking()]
                )
            ),
        ])        

        ls = LaunchService(argv=None)
        ls.include_launch_description(ld)
        return ls.run()

def main():
    rclpy.init(args=None)

    try:    
        booker = TaskBooker()
        rclpy.spin(booker)

    except KeyboardInterrupt:
        booker.destroy_node()
        print("- Booker shutdown -\n")  

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
