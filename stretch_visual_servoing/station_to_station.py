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

    # open machine

    # move from machine op to wafer station
    move(robot, 2.5)

    # pick up wafer

    # move from wafer station to machine load
    move(robot, -1.5)
    rotate(robot, math.pi/2)
    move(robot, 1)
    rotate(robot, math.pi)

    # deposit wafer

    # move from machine load to machine op
    move(robot, -1)
    rotate(robot, math.pi/2)
    move(robot, -1)

    # ideally robot will run the recipe
    # robot opens machine

    # move from machine op to machine unload
    move(robot, 1)
    rotate(robot, -math.pi/2)
    move(robot, 1)

    # withdraw the wafer

    # move from machine unload to wafer station
    rotate(robot, math.pi)
    move(robot, -1)
    rotate(robot, -math.pi/2)
    move(robot, 1.5)

    # place the wafer

    # move from wafer station to machine op
    move(robot, -2.5)

    # loop done