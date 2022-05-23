#Front-left: 13, 16
#Front-right: 6, 12
#Back-left: 26, 21
#Back-right: 19, 20

import rclpy
import time
import RPi.GPIO as GPIO
from rclpy.node import Node
from hcsr04sensor import sensor
from std_msgs.msg import String
from sensor_msgs.msg import Range

class RemoteCar(Node):
    def __init__(self):
      super().__init__('task_car_performer')

      self.controls = [self.forward, self.backwards, self.left, self.right, self.stop]
      self.task_node_name = 'task_car_teleop'

      GPIO.setmode(GPIO.BCM)
      GPIO.setwarnings(False)

      GPIO.setup(13,GPIO.OUT)
      GPIO.setup(16,GPIO.OUT)

      GPIO.setup(6,GPIO.OUT)
      GPIO.setup(12,GPIO.OUT)

      GPIO.setup(26,GPIO.OUT)
      GPIO.setup(21,GPIO.OUT)

      GPIO.setup(19,GPIO.OUT)
      GPIO.setup(20,GPIO.OUT)

      # ultrassons
      self.trig = 3
      self.echo = 4

      #self.testled()
      self.subscription = self.create_subscription(
          String,
          'cmd_car',
          self.listener_callback,
          10)
      self.subscription  # prevent unused variable warning

      #publisher definition
      self.publisher_ = self.create_publisher(Range, 'car_obs_distance', 10)

      self.distance_timer = self.create_timer(0.5, self.ultrassonic_callback)
      self.task_node_timer = self.create_timer(2, self.check_task_node_state)

    def check_task_node_state(self):
      self.get_logger().info('[NODE CHECKER] JUST LOOKING...')     
      for tup in self.get_node_names_and_namespaces():
        if tup[1] == self.get_namespace() and self.task_node_name in tup[0]:
          return

      self.get_logger().info('[TASK NODE DOWN] LOOKING FOR OTHER TASKS')

      raise KeyboardInterrupt

    def ultrassonic_callback(self):
      msg = Range()
      msg.radiation_type = Range.ULTRASOUND
      x = sensor.Measurement
      msg.range = x.basic_distance(self.trig, self.echo)

      self.publisher_.publish(msg)

    def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data)

      value = int(msg.data)

      if value >= 0 and value <= 4:
        self.controls[value]()

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

    def testled(self):
      while True:
        GPIO.output(18, True)
        time.sleep(1)
        GPIO.output(18, False)
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    try:
      car_subscriber = RemoteCar()
      rclpy.spin(car_subscriber)

    except KeyboardInterrupt:
      car_subscriber.destroy_node()

    finally:
      rclpy.shutdown()    
    

if __name__ == '__main__':
    main()