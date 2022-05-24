# ciic_ros_tasks

## Description
Lorem Lorem ...

## Step by step initialization

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
ros2 run task_car tasks_booker --ros-args -r __ns:=/car_1

# Run a remote car control Task node
ros2 run task_car task_car_teleop --ros-args -p my_id:=27 -r __ns:=/"control_car"

# Colocar referências dos config e launch files
```
