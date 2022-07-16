# ciic_ros_tasks
This is a proposal of implementation for a ROS2 task distribution system that also works with microROS based systems.

This project also contains a custom ROS message package, ciic_ros_tasks_messages, created specifically for the task distribution system.

## Step by step initialization

### Create a new directory

Best practice is to create a new directory for every new workspace, so here are the commands to do it.
```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

### Building and sourcing

Run the following commands in your workspace's root

```
# Build the tasks and custom messages packages
colcon build --packages-select ciic_ros_tasks ciic_ros_tasks_interfaces

# Source the built environment
. install/setup.bash
```

### Running/Launching nodes
```
# Run a Booker node
ros2 run ciic_ros_tasks tasks_booker --ros-args -r __ns:=/car_1

# Run a remote car control Task node
ros2 run ciic_ros_tasks task_car_teleop --ros-args -p my_id:=27 -r __ns:=/"control_car"

# Eventually you can use the following commands to launch notes with a preconfigured Python launch file
```
