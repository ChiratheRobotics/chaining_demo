# Chaining Demo
This package is a demonstration of `Chaining Controller` in `ros2_control`.

Tested on: ROS2 Humble

For the Hardware Interface we have used `gazebo_ros2_control` package which contains:

```
1. Gazebo System Interface
2. Gazebo Model Plugin
3. System Interface Implementation
```

In this demo, we will be using effort controller. There are two controllers in total:


```
1. Chaining Controller (At Lower Level)
2. Effort Controller (At Higher Level)
```

# Understanding the Controllers

## 1. Chaining Controller

The chaining controller is exports certain command interfaces by acting as a hardware interface to the higher level controllers which is Effort Controller in our case.

The Chainable Controller Interface has `reference interfaces` which are to be exported to the higher level controller as a `command interface`. There are two key things to note while setting the export of reference interfaces:

1. The `prefix name` of the `reference interfaces` will be the name of of the chaining controller itself as this name would be used by the higher level controllers to form chain to the chaining controller (Refer to Controller Manager to know more about this).

2. The `reference interfaces` are `command interfaces` which have to be populated by the higher level controllers, therefore don't forget to resize this container
(probably in your `on_init` method)

This is how chaining happens in the controllers and this `doesn't require` any change in the configuration file.

Ps: Note that, the chaining controller should be running before you run the higher level controllers as the higher level controllers would be looking for them in their command interfaces to claim.

The chaining controller will clip the effort values from Higher Level Controller and send it as command interface to the hardware interface through the resource manager.

Here is the link to the API of Chaining Controller for reference: https://control.ros.org/humble/doc/api/classcontroller__interface_1_1ChainableControllerInterface.html

## 2. Effort Controller

The effort controller which is at the higher level has 3 `state interfaces` and 1 `command interface` which is an exported `reference interface` by the chaining controller.

The effort controller claims the reference interfaces and then they can be populated by your code. No altercations are required here.

The effort controller's objective is to set random values to the command interfaces received from the chaining controller and update it.

These values will be clipped by the chaining controller.

# Installing and Running

Clone the repository in your colcon workspace. In your home directory:

```
mkdir -p chaining_demo_ws/src && cd chaining_demo_ws/src
```

Clone the repository here and then go back to chaining_demo_ws. Run:
```
colcon build
source install/local_setup.zsh
```
PS: depending on your shell you can change the extension (bash, sh, zsh).

Now run:
```
ros2 launch chaining_controller cart_example_effort.launch.py
```

In a new tab source the workspace and run:
```
ros2 topic echo /joint_states
```
You can see the efforts getting clipped by the chaining controller and being sent as a final command to the resource manager. These values are published by the joint state broadcaster over this topic.
