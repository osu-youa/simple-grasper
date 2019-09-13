How to use:
- Launch the grasper.launch file, which will initialize the UR drivers
  as well as a freedrive node which can be commanded by other scripts
- Run perform_grasp.py, which moves the arm back linearly

<h2>Troubleshooting</h2>

- <b>I'm trying to run the grasp script, but the robot isn't
  responding!</b> First, check the tab which the UR driver is running on
  and see if it says it's received the trajectory. If it has, check on
  the bottom of the teach pendant where it says "Speed 100%" and see if
  there's a "(0%)" next to it. If there is, this is a sign the tablet
  may be stuck in freedrive mode, in which case rebooting the robot
  should work.