# Orochi Hexa

# How to use

In one terminal, run

`roslaunch orochi_hx bringup.launch real_machine:=false simulation:=True headless:=False demo:=False`

In another terminal, run

`rosrun aerial_robot_base keyboard_command.py`

In this terminal, input `r` to arm the quadrotor, then input `t` to takeoff. The quadrotor will takeoff and hover.

Input `l` to land the quadrotor.

## How to use robot_interface for orochi_hx

Instead of `keyboard_command.py`, you can use `robot_interface.py`

In another terminal, run

`rosrun orochi_hx robot_interface.py`
