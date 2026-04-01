import stretch_body.robot as rb
import math

def move(robot, distance: float):
    robot.base.translate_by(distance)
    robot.push_command()
    robot.wait_command()

def rotate(robot, angle: float):
    robot.base.rotate_by(angle)
    robot.push_command()
    robot.wait_command()

def main():
    robot = rb.Robot()
    robot.startup()

    move_amt = 12
    rotate_amt = math.pi/2

    move(robot, -12.0)
    rotate(robot, -rotate_amt)
    move(robot, 1)
    move(robot, -1)
    rotate(robot, rotate_amt)
    move(robot, 4.0)
    rotate(robot, -rotate_amt)
    move(robot, 0.5)
    move(robot, -0.5)
    rotate(robot, rotate_amt)
    move(robot, 8.0)
    robot.base.set_velocity(0.0, 0.0)
    robot.push_command()

if __name__ == '__main__':
    main()
