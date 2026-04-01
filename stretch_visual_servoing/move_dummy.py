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

    move_amt = 4
    rotate_amt = math.pi/2

    move(robot, -move_amt)
    rotate(robot, rotate_amt)
    rotate(robot, -rotate_amt)
    move(robot, move_amt)
