import rclpy, time, signal, sys, threading

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread

from getkey import getkey, keys

from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Range
from ciic_ros_tasks_messages.msg import TaskBooking

class TaskAdvertiser(Node):
    def __init__(self, task):
        super().__init__('task_advertiser')

        self.get_logger().info("Advertiser Launched")

        self.task = task

        #default values are set to id=1
        #if the param file isn't specified on launch, this values are used
        self.declare_parameters(
            namespace='',
            parameters=[
                ('my_id', 1)
            ])
        
        self.task_namespace = self.get_namespace()
        self.id = self.get_parameter('my_id').value

        #publisher definition
        self.publisher_ = self.create_publisher(TaskBooking, '/tasks_booking', 10)

        #subscriber definition
        self.subscription = self.create_subscription(
            TaskBooking,
            '/tasks_booking',
            self.listener_callback,
            10)
        self.subscription       

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if(self.task.booked):
            return

        msg = TaskBooking()
        msg.sender_id = self.id
        msg.task_namespace = self.task_namespace
        msg.action = 'A'
        self.publisher_.publish(msg)
        self.get_logger().info('SENDING | SENDER_ID:"%d" | NAMESPACE:"%s" | ACTION:"%s"' % (msg.sender_id, msg.task_namespace, msg.action))

    def listener_callback(self, msg):
        if(msg.action == 'A' or msg.action == 'S' or self.id != msg.recipient_id):
            return

        self.get_logger().info('RECEIVED | SENDER_ID:"%d" | NAMESPACE:"%s" | ACTION:"%s"' % (msg.sender_id, msg.task_namespace, msg.action))
        
        msg_to_send = TaskBooking()
        msg_to_send.task_namespace = self.task_namespace
    
        if(not self.task.booked):

            #verificar se o namespace vem vazio. se sim é microcontrolador
            #variavel dentro da task
            if(msg.performer_identifier == 'microros'):
                self.task.microcontrolador = True
                self.task.micro_ns = msg.task_namespace
            else:
                msg_to_send.performer_identifier = self.task.performer_filename

            msg_to_send.action = 'S'            
            self.task.booked =  True
        else:
            msg_to_send.action = 'F'
            
        msg_to_send.sender_id = self.id
        msg_to_send.recipient_id = msg.sender_id

        self.publisher_.publish(msg_to_send)
        self.get_logger().info('SENDING | SENDER_ID:"%d" | RECIPIENT_ID:"%d" | NAMESPACE:"%s" | ACTION:"%s"' % (msg_to_send.sender_id, msg_to_send.recipient_id, msg_to_send.task_namespace, msg_to_send.action))

class TaskCarTeleop(Node):
    def __init__(self):
        super().__init__('task_car_teleop')
        self.get_logger().info("Task Node Launched")

        self.received_req = True
        self.task_running = False
        self.booked = False
        self.performer_filename = 'task_car_performer'
        self.microcontrolador = False
        self.micro_ns = ''

        #timer de execução de código cliente da tarefa
        #executa uma vez assim que o advertiser registar a reserva

        signal.signal(signal.SIGINT, lambda s,f: self.on_ctrl_c())

        self.timer = self.create_timer(0.2, self.timer_callback)

    def on_ctrl_c(self):
        if not self.booked:
            exit(0)
        
        self.get_logger().info("You pressed CTRL+C. Please press Q to exit.")           
        pass

    def timer_callback(self):
        if not self.booked:
            return

        self.timer.cancel()
        self.curr_distance = 0

        if not self.microcontrolador:
            self.subscription = self.create_subscription(
                Range,
                'car_obs_distance',
                self.distance_callback,
                10)
            self.subscription

        self.display_information()

        self.t = threading.Thread(target=self.handle_control)
        self.t.daemon = True
        self.t.start()
        self.t.join()

        exit(0)

    def receive_ping(self, msg):
        self.get_logger().info("Received ping from pair node")
        self.publisher_ping = self.create_publisher(Int32, self.micro_ns + '/ping', 10)

        msg_to_send = Int32()
        msg_to_send.data = 1
        self.publisher_ping.publish(msg_to_send)        

    def handle_control(self):
        ##verificar se é micro controlador
        control_car_topic = self.micro_ns + '/cmd_car' if self.microcontrolador else 'cmd_car'
        ping_topic = self.micro_ns + '/ping' if self.microcontrolador else 'ping'

        self.publisher_ = self.create_publisher(String, control_car_topic, 10)

        self.subscription_ping = self.create_subscription(
                Int32,
                ping_topic,
                self.receive_ping,
                10)
        self.subscription_ping

        while True:
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
            elif key == 'q' or key == 'Q':
                self.get_logger().info("User concluded Task. Stopping task and advertiser node...\n\n")
                break
            else:
                self.get_logger().info("Remember to press one of the WASD keys or the SPACEBAR")

    def display_information(self):
        print("\nIn order to control the car, take a look at the following instructions:")
        print("Pressing one of the 'keys' will make the car go in the associated 'direction'\n")
        print("KEY | DIRECTION")
        print("\n\nW   | Forward\nS   | Backwards\nA   | Left\nD   | Right\n\n")
        print("Press the SPACEBAR to stop the car completely\n\nPress [Q] to exit the program\n\n")
        print("---------------------------------------------------------------------------------\n\n")

        return

    def perform_control(self, value):
        msg = String()
        msg.data = value
        self.publisher_.publish(msg)

    def distance_callback(self, msg):
        print("\t\t\tCLOSEST OBJECT AT: {} cm".format(self.curr_distance), end="\r")       
        self.curr_distance = msg.range

def main():
    rclpy.init(args=None)

    try:
        task_node = TaskCarTeleop()
        adviser_node = TaskAdvertiser(task_node)

        executor = SingleThreadedExecutor()
        executor.add_node(task_node)
        executor.add_node(adviser_node)
        
        try:
            executor.spin()
            
        finally:
            executor.shutdown()
            task_node.destroy_node()
            adviser_node.destroy_node()
    
    finally:
        rclpy.shutdown()    

if __name__ == '__main__':
    main()
