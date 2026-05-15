import stretch_body.robot as rb
import math
import time

FROM_MACHINE_TO_WTABLE = -1.9
QUARTER_COUNTERCLOCK = math.pi/2
QUARTER_CLOCK = -QUARTER_COUNTERCLOCK
NEXT_TO_MACHINE_OR_TRAY = 0.80
FROM_WTABLE_TO_TRAY = 0.85
FROM_TRAY_TO_MACHINE = 1.05

def default_gripper(robot):
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.end_of_arm.move_to('wrist_roll', 0.0)
    robot.end_of_arm.move_to('stretch_gripper', 50) # [-100, 100] => [fully closed, fully open]
    robot.push_command()
    robot.wait_command()

def move(robot, distance: float):
    robot.base.translate_by(distance, v_m=0.5)
    robot.push_command()
    print(f"waited for move by {distance}m command? {robot.wait_command(timeout=60.0)}")

def rotate(robot, angle: float):
    robot.base.rotate_by(angle)
    robot.push_command()
    print(f"waited for move by {angle}m command? {robot.wait_command()}")

def do_dw_pupd(robot, dw=True, put_down=False):
    STOW_ANGLE = (3 * math.pi)/4
    PUPD_DIST = 0.03

    # deposit/withdraw values
    DW_HEIGHT = 1.05
    DW_LENGTH = 0.25
    PUPD_DIST = 0.02

    if not dw:
        # pick up/put down values
        DW_HEIGHT = 0.98
        DW_LENGTH = 0.3
        PUPD_DIST = 0.08

    print(f"{DW_HEIGHT}, {DW_LENGTH}, {PUPD_DIST}")
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.lift.move_to(DW_HEIGHT)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', 0.0)
    time.sleep(2)

    robot.arm.move_to(DW_LENGTH)
    robot.push_command()
    robot.wait_command()
    
    robot.lift.move_by(-PUPD_DIST, v_m=0.02)
    robot.push_command()
    robot.wait_command()

    if put_down:
        robot.end_of_arm.move_to('stretch_gripper', -70) # [-100, 100] => [fully closed, fully open]
        robot.push_command()
        robot.wait_command()
        time.sleep(1)
        robot.end_of_arm.move_to('stretch_gripper', 50) # [-100, 100] => [fully closed, fully open]
        robot.push_command()
        robot.wait_command()

    robot.lift.move_by(PUPD_DIST)
    robot.push_command()
    robot.wait_command()

    robot.arm.move_to(0.0)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)

def main():
    robot = rb.Robot()
    robot.startup()

    # open machine

    # move from machine op to wafer station
    default_gripper(robot)
    move(robot, FROM_MACHINE_TO_WTABLE)
    rotate(robot, QUARTER_CLOCK)
    move(robot, NEXT_TO_MACHINE_OR_TRAY)
    rotate(robot, QUARTER_COUNTERCLOCK)
    # pick up wafer
    do_dw_pupd(robot, False)

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
    do_dw_pupd(robot, True, True)

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