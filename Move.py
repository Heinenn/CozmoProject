import cozmo
import sys
import logging
import getopt
import time

from cozmo.util import degrees, distance_mm, speed_mmps


# Logger
log = logging.getLogger('ok.FollowLine')

#constants
speed = 200

#Cozmo-Modes
MODE_STEP = 1
MODE_DRIVE = 2




def drive_stop(robot: cozmo.robot.Robot):
    """ Start driving
    """
    log.info('Stop...')
    robot.stop_all_motors()


def drive(robot: cozmo.robot.Robot):
    """ Start driving
    """
    log.info('Drive...')
    robot.drive_wheel_motors(speed, speed)


def drive_left(robot: cozmo.robot.Robot):
    """ Drive to left
    """
    log.info('Drive left...')
    robot.drive_wheel_motors(speed - 200, speed)


def drive_backwards(robot: cozmo.robot.Robot):
    """ Start driving
    """
    log.info('Drive backwards...')
    robot.drive_wheel_motors(int(0-int(speed)),int(0-int(speed)))


def drive_right(robot: cozmo.robot.Robot):
    """ Drive to right
    """
    log.info('Drive right...')
    robot.drive_wheel_motors(speed, speed - 200)


def move_right(robot: cozmo.robot.Robot, turndegree, move):
    """ Moves the robot to the right, in case the path is too far on the right
    """
    log.info('Move right...')
    robot.turn_in_place(degrees(int(0-int(turndegree)))).wait_for_completed()
    robot.drive_straight(distance_mm(move), speed_mmps(speed)).wait_for_completed()
    #robot.turn_in_place(degrees(int(turndegree))).wait_for_completed()
    #disable above comment for turn in originally line of sight


def move_left(robot: cozmo.robot.Robot, turndegree, move):
    """ Moves the robot to the left, in case the path is too far on the right
    """
    log.info('Move left...')
    robot.turn_in_place(degrees(int(turndegree))).wait_for_completed()
    robot.drive_straight(distance_mm(move), speed_mmps(speed)).wait_for_completed()
    #robot.turn_in_place(degrees(int(0-int(turndegree)))).wait_for_completed()
    #disable above comment for turn in originally line of sight


def correct_turn(robot: cozmo.robot.Robot, turndegree, drivemode):
    """ Performs the turn for the correction. In case of
       'drive' mode, wheels are stopped and started again,
       in case of 'step' mode, only turn is performed
    """
    log.info('Turn for correction...')
    if drivemode == MODE_DRIVE:
        robot.stop_all_motors()
    robot.turn_in_place(degrees(int(turndegree))).wait_for_completed()
    if drivemode == MODE_DRIVE:
        robot.drive_wheel_motors(speed, speed)


def cozmo_cli(robot: cozmo.robot.Robot):
    """ Main loop implementing simplistic CLI
    """
    log.info('Entering Cozmo program')
    robot.stop_all_motors()
    robot.set_head_light(True)
    #robot.camera.image_stream_enabled = True
    robot.set_lift_height(1.0).wait_for_completed()
    while True:
        run_cmd = input('C>')
        if run_cmd == 'q':
            drive_stop(robot)
        if run_cmd == 'w':
            drive(robot)
        if run_cmd == 'a':
            drive_left(robot)
        if run_cmd == 's':
            drive_backwards(robot)
        if run_cmd == 'd':
            drive_right(robot)
        if run_cmd == 'sleep':
            time.sleep(1)
        if 'left' in run_cmd:
            cliinput = run_cmd.split(' ')
            turndegree = cliinput[1]
            distance = cliinput[2]
            move_left(robot, turndegree, distance)                          #correct_turn(Cozmo, degrees to turn, distance to travel in mm)
        if 'right' in run_cmd:
            cliinput = run_cmd.split(' ')
            turndegree = cliinput[1]
            distance = cliinput[2]
            move_right(robot, turndegree, distance)                         #correct_turn(Cozmo, degrees to turn, distance to travel in mm)
        if 'correct' in run_cmd:
            turndegree = ''.join(filter(lambda x: x.isdigit(), run_cmd))
            correct_turn(robot, turndegree, 1)                              #correct_turn(Cozmo, degrees to turn, DriveMode(0=step;1=drive))


def main(argv):
    # Set-up logging
    formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s')
    handler = logging.StreamHandler()
    handler.setLevel(logging.INFO)
    handler.setFormatter(formatter)
    log.setLevel(logging.INFO)
    log.addHandler(handler)
    # Eval command line
    usecase = 'cli'
    try:
        opts, args = getopt.getopt(argv, 'hu:', ['usecase='])
    except getopt.GetoptError:
        print('line_follow.py -u <usecase>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('line_follow.py -u <usecase>')
            sys.exit()
        elif opt in ("-u", "--usecase"):
            usecase = arg
    log.info('Executing use case ' + usecase)
    if usecase == 'cli':
        cozmo.run_program(cozmo_cli)


if __name__ == '__main__':
    main(sys.argv[1:])