import rclpy, threading, signal

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread
from std_msgs.msg import String
from threading import Thread

from getkey import getkey, keys

class Teste(Node):
    def __init__(self):
        super().__init__('teste')

        self.get_logger().info("Teste Launched")
        self.ns = '/control_car'

        #publisher definition
        self.publisher_ = self.create_publisher(String, '/t3', 10)

        #subscriber definition
        self.subscription = self.create_subscription(
            String,
            '/t1',
            self.t1_callback,
            10)
        self.subscription

        self.timer = self.create_timer(3, self.timer_callback)
        self.i = 0
        self.exit = False

        self.timer_h = self.create_timer(0.2, self.timer_callback_h)

    def timer_callback_h(self):
        self.timer_h.cancel()
        self.get_logger().info("timer :)")

        self.t = threading.Thread(target=self.handle_control)
        self.t.daemon = True
        self.t.start()
        self.t.join()

        exit(0)

    def t1_callback(self,msg):
        self.get_logger().info("T1")

    def t2_callback(self,msg):
        self.get_logger().info("T2")

    def timer_callback(self):
        msg = String()
        msg.data = 'P'
        self.publisher_.publish(msg)
        self.get_logger().info('publishing in t3')

    def handle_control(self):
        #subscriber definition
        self.subscription = self.create_subscription(
            String,
            '/t2',
            self.t2_callback,
            10)
        self.subscription

        self.get_logger().info('Thread')

        while True:
            self.get_logger().info("inside loop")
            key = getkey()
            if key == 'w' or key == 'W':
                self.perform_control('0')
            elif key == 's' or key == 'S':
                self.perform_control('1')
            elif key == 'a' or key == 'A':
                self.perform_control('2')
            elif key == 'd' or key == 'D':
                self.perform_control('3')
            elif key == keys.SPACE:
                self.perform_control('4')
            elif key == 'q':
                self.get_logger().info("EXIT")                
                break
            else:
                self.get_logger().info("Remember to press one of the WASD keys or the SPACEBAR")

        self.get_logger().info("ended loop")
        
        return

def main():
    rclpy.init(args=None)
    task_node = Teste()

    try:
        executor = SingleThreadedExecutor()
        executor.add_node(task_node)
    
        try:
            executor.spin()
            
        finally:
            executor.shutdown()
            task_node.destroy_node()
    
    finally:
        rclpy.shutdown()     

if __name__ == '__main__':
    main()