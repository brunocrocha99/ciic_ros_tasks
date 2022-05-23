import rclpy

from rclpy.node import Node

from launch import LaunchDescription
from launch import LaunchService
import launch_ros.actions

from launch.actions import (EmitEvent, RegisterEventHandler, LogInfo)
from launch.event_handlers import (OnProcessExit, OnProcessIO)
from launch.events import Shutdown

class Teste(Node):
    def __init__(self):
        super().__init__('teste')

        self.get_logger().info("Teste Launched")
        self.ns = '/control_car'
        
        self.launch_node()

    def launch_node(self):
        perfomer_node = launch_ros.actions.Node(package = 'task_car', executable='task_car_performer', output='screen', namespace='control_test')

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
            
        ])        

        ls = LaunchService(argv=None)
        ls.include_launch_description(ld)
        return ls.run()
        

def main():
    rclpy.init(args=None)

    task_node = Teste()

    rclpy.spin(task_node)

    task_node.destroy_node()
    
    rclpy.shutdown()    

if __name__ == '__main__':
    main()