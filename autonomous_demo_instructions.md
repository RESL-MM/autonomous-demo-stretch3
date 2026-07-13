### Stretch3 Autonomous Demo Instructions (13-07)

### Important Things to Note
- Only move the robot manually when it is in runstop mode (i.e. the battery indicator and runstop lights are flashing)
- Make sure all additional hardware (e.g. CAD end-effector) is removed when homing the robot and performing the system check
- When finished, make sure the robot is fully turned off and is securely plugged into the docking station and is charging (the charger should react and change from its previous state if it is plugged in properly)

### Pre-requisites/Dependencies
- [XQuartz](https://www.xquartz.org/) (if controlling and SSHing into robot using MacOS)
- Qt (install using [these instructions](https://web.stanford.edu/dept/cs_edu/resources/qt/install-windows), the robot will throw an error if Qt is not installed properly on the machine SSHing into the Stretch3)

### Pre-Demo Setup
- Powering on the robot
	- Move the robot from the docking station to an open space (since it is still powered off and the wheels are unlocked, runstop checks are not required here)
	- Flip the power switch located at the base of the robot near its USB and I/O Ports; the robot should make a beeping noise once it is fully powered on
	- Make sure all the end-effectors are detached and the robot is in a space where it can move, extend, and rotate freely later when homing
- Connect to the Robot
	- Once the robot is powered on, connect to the stretch3 wifi network
	- SSH into the robot using the following command:  ```ssh -X hello-robot@192.168.0.124```
- Home and Perform a System Check on the Robot
	- The maintenance scripts are global and can be called from any directory on the robot
	- In general, before running any scripts or processes, it is a good habit to run ```stretch_free_robot_processes.py```
	- To home and perform a system check, run the following commands **(wait for confirmation in the terminal that the previous command is finished before running the next command)**:
		- ```stretch_free_robot_processes.py```
		- ```stretch_robot_home.py```
		- ```stretch_free_robot_processes.py```
		- ```stretch_robot_system_check.py```
		- ```stretch_free_robot_processes.py```
	- **Some things to note:**
		- the maintenance scripts can be auto completed by hitting tab after enough of the command has been entered, so it's not necessary to type out the entire thing every time
		- there may be some warnings or out of date packages brought up by the system check, those are not an issue for now and can be ignored
- Prepare the robot for a demo iteration
	- Attach the additional hardware necessary for the demo to the Stretch3 (i.e. the CAD end-effector and the vacuum wand)
		- The cabling with the vacuum wand can be a little troublesome, I've found keeping the vacuum bank on the base of the robot and running the wire along the arm and on top of the wrist to the pen as being the best way of managing the cable during operation, however as long as it's set up in a way that **the wire does not get caught in joints or tangled through movement** and **the gripper camera is not obstructed (i.e. the left and right fingertip tags and the RIE80 tags should not be obscured)**
		- If you want to do a full demo with the wafer actually being picked up and deposited, as opposed to simply seeing where the pen contacts the wafer and where it contacts the RIE80 tray (dry run), I suggest keeping the vacuum wand power cable on hand and only plugging it in (while still keeping the power cable out of the Stretch3's path) for the duration of the demo where the vacuum wand is used-- there is a high tripping and danger hazard present when the power cable is left loose as it most likely will get tangled and/or get caught in the wheels of the Stretch3
	- Move the robot to an appropriate starting point for the demo (the robot should be facing toward the cleanroom entrance area, the arm should be roughly perpendicular to and extend towards the RIE80, and the robot should be placed a few feet behind and roughly in between the two localisation tags on the floor); **make sure all movements and adjustments are done when the robot is in runstop mode**
- Misc. Environment Setup
	- Ensure that no ArUco tag is occluded (in particular the two tags on the base, the two tags on the floor, and the tags on the tags on the RIE80)
	- Clear a path from the machine to the table near the RIE80 and clear the side of the RIE80 (refer to [this video](https://drive.google.com/file/d/1YR1qLlcrEJYWG033WLnpk7WMhg7hJebS/view?usp=drive_link) for what that roughly looks like)-- the robot will pick up and deposit the wafer near the left corer of the desk
	- The demo assumes the RIE80 dial is in the closed position, **make sure it is in the closed position before the demo starts**
	- **Do all the necessary setup for the RIE80 so it is able to be operated by the robot (i.e. log in and vent the machine so it can be opened and closed)**

### Demo Setup and Running
- Starting the Demo
	- In the SSH terminal, navigate to the demo directory and run the demo script; i.e. run the following:
		- ```cd ~/repos/autonomous-demo-stretch3/complete_demo```
		- ```python3 main.py```
- Demo Running
	- As it stands, the demo will run for 1 complete iteration and then the program will exit if all goes well
	- Keep a close watch on the robot, if something goes wrong or looks like it is about to either immediately stop the program (ctrl + c in the process terminal) or press the runstop button THEN stop the program (the latter is easier)
		- If something goes wrong there are two options: fully restart the robot or soft restart
		- fully restart:
			- safer of the two options, less likely for any carry over errors, but means needing to power off and restart the robot (perform all steps from Pre-Demo onwards again)
		- soft restart:
			- once everything has stopped running, move the robot back to the start of the demo position, and rerun the main.py script
				- optionally the robot can be homed again, but that requires removing the end-effector hardware

### Post-Demo Cleanup and Shutdown
- Once the demo is completed, run the following command to shutdown the robot ```sudo shutdown -h now```
- Then flip the power switch to turn off the robot computer
- Move the robot to the docking station and ensure that it is being charged (the charger should change its display once properly plugged in)
- Manually lower the robot arm and stow the wrist somewhere along the base in such a way that it is not in an awkward position or under unnecessary mechanical stress
	- Once powered off the robot arm will slowly descend, it's better to manually place the arm in a resting position since it might settle into an awkward position (given the CAD tool is connected) if left to descend on its own
- Do the usual cleanup routine for the environment (i.e. pump and chamber clean the RIE80 and place back any moved equipment and furniture)
