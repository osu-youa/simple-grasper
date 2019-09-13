How to use:
- Launch the grasper.launch file, which will initialize the UR drivers
  as well as a freedrive node which can be commanded by other scripts
- Run perform_grasp.py, which moves the arm back linearly

For an example of how to analyze the resulting pickle files, run the
analyze_results.py file with a generated file (e.g. rosrun apple_grasper
analyze_results.py grasper_20190913151006.pickle)

<b>Be warned!</b> The current trajectory is done using an IK solver
which I randomly found online; this could result in weird movement near
singularities. <b>KEEP YOUR HAND ON THE RED BUTTON AT ALL TIMES!</b>

<h2>Troubleshooting</h2>

- <b>I'm trying to run the grasp script, but the robot isn't
  responding!</b> First, check the tab which the UR driver is running on
  and see if it says it's received the trajectory. If it has, check on
  the bottom of the teach pendant where it says "Speed 100%" and see if
  there's a "(0%)" next to it. If there is, this is a sign the tablet
  may be stuck in freedrive mode, in which case rebooting the robot
  should work.