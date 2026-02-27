import subprocess
import sys
import time
import stretch_body.robot as rb
import argparse
import d405_helpers as dh


def move_base_back(distance_m=0.25):
    robot = rb.Robot()
    robot.startup()

    robot.base.translate_by(-distance_m)
    robot.push_command()
    robot.wait_command()

    robot.stop()
    
def move_base_forward(distance_m=0.25):
    robot = rb.Robot()
    robot.startup()

    robot.base.translate_by(distance_m)
    robot.push_command()
    robot.wait_command()

    robot.stop()


def main():
    parser = argparse.ArgumentParser(
        prog='Stretch 3 Full Demo',
        description='Runs the dial twisting demo followed by the button pressing demo.',
    )
    parser.add_argument(
        '-e', '--exposure', action='store', type=str, default='low',
        help=f'Set the D405 exposure to {dh.exposure_keywords} or an integer in the range {dh.exposure_range}',
    )
    args = parser.parse_args()
    exposure = args.exposure

    if not dh.exposure_argument_is_valid(exposure):
        raise argparse.ArgumentTypeError(
            f'The provided exposure setting, {exposure}, is not a valid keyword, '
            f'{dh.exposure_keywords}, or is outside of the allowed numeric range, {dh.exposure_range}.'
        )

    print('=== Starting Dial Twisting Demo ===')
    result = subprocess.run(
        [sys.executable, 'twist_dial_demo.py', '-e', exposure],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Dial twisting demo exited with code {result.returncode}')

    print('=== Moving base back 12 cm ===')
    move_base_back(0.12)

    print('=== Pausing for 1 second ===')
    time.sleep(1.0)

    print('=== Starting Button Pressing Demo ===')
    result = subprocess.run(
        [sys.executable, 'button_pressing_demo.py', '-e', exposure],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Button pressing demo exited with code {result.returncode}')
    
    print('=== Moving base forward 12 cm ===')
    move_base_forward(0.12)

    print('=== Full Demo Complete ===')


if __name__ == '__main__':
    main()
