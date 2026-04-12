import stretch_body.robot as rb
import math

FROM_MACHINE_TO_WTABLE = -2.5
QUARTER_COUNTERCLOCK = math.pi/2
QUARTER_CLOCK = -QUARTER_COUNTERCLOCK
NEXT_TO_MACHINE_OR_TRAY = 0.9
FROM_WTABLE_TO_TRAY = 1.4
FROM_TRAY_TO_MACHINE = 1.1

def move(robot, distance: float):
    robot.base.translate_by(distance, v_m=0.25)
    robot.push_command()
    print(f"waited for move by {distance}m command? {robot.wait_command(timeout=60.0)}")

def rotate(robot, angle: float):
    robot.base.rotate_by(angle)
    robot.push_command()
    print(f"waited for move by {angle}m command? {robot.wait_command()}")

def main():
    robot = rb.Robot()
    robot.startup()

    # open machine

    # move from machine op to wafer station
    robot.stow()
    move(robot, FROM_MACHINE_TO_WTABLE)
    rotate(robot, QUARTER_CLOCK)
    move(robot, NEXT_TO_MACHINE_OR_TRAY)
    rotate(robot, QUARTER_COUNTERCLOCK)
    # pick up wafer

    # move from wafer station to machine load
    rotate(robot, QUARTER_CLOCK)
    move(robot, -NEXT_TO_MACHINE_OR_TRAY)
    rotate(robot, QUARTER_COUNTERCLOCK)
    move(robot, FROM_WTABLE_TO_TRAY)
    rotate(robot, QUARTER_CLOCK)
    move(robot, NEXT_TO_MACHINE_OR_TRAY)
    rotate(robot, QUARTER_COUNTERCLOCK)
    rotate(robot, QUARTER_COUNTERCLOCK)
    # deposit wafer

    # move from machine load to machine op
    rotate(robot, QUARTER_CLOCK)
    rotate(robot, QUARTER_CLOCK)
    move(robot, -NEXT_TO_MACHINE_OR_TRAY)
    rotate(robot, QUARTER_COUNTERCLOCK)
    move(robot, FROM_TRAY_TO_MACHINE)

    ### ideally robot will run the recipe ###
    
    # robot opens machine

    # move from machine op to machine unload

    # withdraw the wafer

    # move from machine unload to wafer station

    # place the wafer

    # move from wafer station to machine op

    ### loop done ###

if __name__ == '__main__':
    main()