import stretch_body.robot as rb
import math
import time

def move(robot, distance: float):
    robot.base.translate_by(distance, v_m=0.3)
    robot.push_command()
    print(robot.wait_command(timeout=120.0))

def rotate(robot, angle: float):
    robot.base.rotate_by(angle)
    robot.push_command()
    print(robot.wait_command())

def main():
    robot = rb.Robot()
    robot.startup()

    move_amt = 2.5
    rotate_amt = math.pi/2 # counterclockwise rotation

    robot.stow()

    move(robot, -move_amt)
    rotate(robot, -rotate_amt)
    move(robot, 0.9)
    rotate(robot, rotate_amt)
    time.sleep(2)
    rotate(robot, -rotate_amt)
    move(robot, -0.9)
    rotate(robot, rotate_amt)
    move(robot, 1.4)
    rotate(robot, -rotate_amt)
    move(robot, 0.9)
    rotate(robot, rotate_amt)
    rotate(robot, rotate_amt)
    time.sleep(2)
    rotate(robot, -rotate_amt)
    rotate(robot, -rotate_amt)
    move(robot, -0.9)
    rotate(robot, rotate_amt)
    move(robot, 1.1)

if __name__ == '__main__':
    main()
