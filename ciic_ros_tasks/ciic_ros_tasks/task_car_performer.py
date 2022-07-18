import rclpy
import time
import RPi.GPIO as GPIO
from rclpy.node import Node
from hcsr04sensor import sensor
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
from rclpy.executors import MultiThreadedExecutor

class RemoteCar(Node):
    def __init__(self):
      super().__init__('task_car_performer')

      self.controls = [self.forward, self.backwards, self.left, self.right, self.stop]
      self.task_node_name = 'task_car_teleop'

      GPIO.setmode(GPIO.BCM)
      GPIO.setwarnings(False)

      #wheels' ports definitions
      #Front-left: 13, 16
      GPIO.setup(13,GPIO.OUT)
      GPIO.setup(16,GPIO.OUT)

      #Front-right: 6, 12
      GPIO.setup(6,GPIO.OUT)
      GPIO.setup(12,GPIO.OUT)

      #Back-left: 26, 21
      GPIO.setup(26,GPIO.OUT)
      GPIO.setup(21,GPIO.OUT)

      #Back-right: 19, 20
      GPIO.setup(19,GPIO.OUT)
      GPIO.setup(20,GPIO.OUT)

      # ultrassound ports
      self.trig = 3
      self.echo = 4

      #topic to receive controls
      self.subscription = self.create_subscription(
          String,
          'cmd_car',
          self.listener_callback,
          10)
      self.subscription  # prevent unused variable warning

      #publisher definition
      self.publisher_ = self.create_publisher(Range, 'car_obs_distance', 10)
      self.publisher_ping_ = self.create_publisher(Int32, 'ping', 10)

      self.distance_timer = self.create_timer(0.5, self.ultrassonic_callback)
      self.task_node_timer = self.create_timer(2, self.check_task_node_state)

      self.subscription_ping = self.create_subscription(
        Int32,
        'ping',
        self.receive_ping,
        10)
      self.subscription_ping

    #stop performing task if pair task node is shutdown
    def check_task_node_state(self):
      #self.get_logger().info('[NODE CHECKER] JUST LOOKING...')     
      for tup in self.get_node_names_and_namespaces():
        if tup[1] == self.get_namespace() and self.task_node_name in tup[0]:
          return

      #self.get_logger().info('[TASK NODE DOWN] LOOKING FOR OTHER TASKS')

      raise KeyboardInterrupt

    #publish real time registered distance
    def ultrassonic_callback(self):
      msg = Range()
      msg.radiation_type = Range.ULTRASOUND
      x = sensor.Measurement
      msg.range = x.basic_distance(self.trig, self.echo)

      self.publisher_.publish(msg)

    #call motion control function based on the received value
    def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data)

      value = int(msg.data)

      if value >= 0 and value <= 4:
        self.controls[value]()

    def receive_ping(self, msg):
      self.get_logger().info("Ping received")
      self.get_logger().info("sending ping")

      msg_to_send = Int32()
      msg_to_send.data = 1
      publisher_ping_.publish(msg_to_send)

    def forward(self):
      self.get_logger().info("Moving forward...")
      GPIO.output(13, True)
      GPIO.output(16, False)
      GPIO.output(6, True)
      GPIO.output(12, False)
      GPIO.output(26, True)
      GPIO.output(21, False)
      GPIO.output(19, True)
      GPIO.output(20, False)

    def backwards(self):
      self.get_logger().info("Moving backwards...")
      GPIO.output(13, False)
      GPIO.output(16, True)
      GPIO.output(6, False)
      GPIO.output(12, True)
      GPIO.output(26, False)
      GPIO.output(21, True)
      GPIO.output(19, False)
      GPIO.output(20, True)

    def stop(self):
      self.get_logger().info("Stopping car...")
      GPIO.output(13, False)
      GPIO.output(16, False)
      GPIO.output(6, False)
      GPIO.output(12, False)
      GPIO.output(26, False)
      GPIO.output(21, False)
      GPIO.output(19, False)
      GPIO.output(20, False)

    def left(self):
      self.get_logger().info("Turning left...")
      GPIO.output(13, False)
      GPIO.output(16, False)
      GPIO.output(6, True)
      GPIO.output(12, False)
      GPIO.output(26, False)
      GPIO.output(21, False)
      GPIO.output(19, True)
      GPIO.output(20, False)

    def right(self):
      self.get_logger().info("Turning right...")
      GPIO.output(13, True)
      GPIO.output(16, False)
      GPIO.output(6, False)
      GPIO.output(12, False)
      GPIO.output(26, True)
      GPIO.output(21, False)
      GPIO.output(19, False)
      GPIO.output(20, False)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    car_subscriber = RemoteCar()
    executor.add_node(car_subscriber)

    try:      
      executor.spin()

    except KeyboardInterrupt:
      car_subscriber.destroy_node()

    finally:
      rclpy.shutdown()    
    

if __name__ == '__main__':
    main()