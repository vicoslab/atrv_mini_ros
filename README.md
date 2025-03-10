# iRobot ATRV Mini

A perfectly cromulent robot from a less civilized age.

![image](https://github.com/user-attachments/assets/af1fb5bc-297d-4e12-a28d-f3bd25570b96)

## Usage

Connect the ATRV RFlex board to `/dev/ttyUSB0` and run:

```bash 
roslaunch atrv_mini_ros atrv.launch
```
Use the RFlex GUI on the robot to turn off brakes, then /cmd_vel commands will be executed.

Available topics (some of them don't really work):

```bash 
/atrv_node/battery_state
/atrv_node/brake_power
/atrv_node/bump
/atrv_node/cmd_accel
/atrv_node/cmd_brake_power
/atrv_node/cmd_sonar_power
/atrv_node/cmd_vel
/atrv_node/odom
/atrv_node/sonar_cloud_base
/atrv_node/sonar_cloud_body
/atrv_node/sonar_power
/atrv_node/state
/atrv_node/twist
```
The update rate has been reduced to 15 Hz and state update requests to 0.5 Hz, to reduce procesing load on the 30 year old microcontroller and increase robot responsiveness.


## Random Info Section

### Power draw

The RFlex board seems to draw 15W when idling, and the motor drivers draw an extra 10W when the brake is off.
