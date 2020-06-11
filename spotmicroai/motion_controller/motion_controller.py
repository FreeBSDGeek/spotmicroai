import signal
import sys

import queue
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time
import inspect

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config
import spotmicroai.utilities.queues as queues
from spotmicroai.utilities.general import General

log = Logger().setup_logger('Motion controller')

class MyServo:
    def __init__(self, channel, min_pulse=500, max_pulse=2500):
        self._desciption = ""
        self._currentangle = None
        self._channel = channel
        self._rest = None
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self._servo = servo.Servo(channel)
        self._servo.set_pulse_width_rage(min_pulse=self.min_pulse, max_pulse=self.max_pulse)
        self._newangle = self._currentangle

    @property
    def description(self):
        return self._description

    @description.setter
    def description(self,desc):
        self._description = desc

    @property
    def currentangle(self):
        return self._currentangle

    @currentangle.setter
    def currentangle(self,a):
        self._currentangle = a

    @property
    def channel(self):
        return self._channel

    @channel.setter
    def channel(self,ch):
        self._servo = servo.Servo(ch, self.min_pulse, self.max_pulse)
        self._channel = ch

    @property
    def rest(self):
        return self._rest

    @rest.setter
    def rest(self,r):
        self._rest = r

    @property
    def newangle(self):
        return self._newangle

    @new_angle.setter
    def newangle(self,a):
        self._newangle = a

    @property
    def delta(self):
        return self._currentangle - self._newangle

    @property
    def servo(self):
        return self._servo

    def move(self):
        if self._newangle != self._currentangle:
            self._servo.angle = self._newangle
            time.sleep(0.02)
            self._currentangle = self._newangle







class MotionController:
    boards = 1

    is_activated = False

    i2c = None
    pca9685_1 = None
    pca9685_2 = None

    pca9685_1_address = None
    pca9685_1_reference_clock_speed = None
    pca9685_1_frequency = None
    pca9685_2_address = None
    pca9685_2_reference_clock_speed = None
    pca9685_2_frequency = None

    servo_rear_shoulder_left = None
    servo_rear_shoulder_left_pca9685 = None
    servo_rear_shoulder_left_channel = None
    servo_rear_shoulder_left_min_pulse = None
    servo_rear_shoulder_left_max_pulse = None
    servo_rear_shoulder_left_rest_angle = None

    servo_rear_leg_left = None
    servo_rear_leg_left_pca9685 = None
    servo_rear_leg_left_channel = None
    servo_rear_leg_left_min_pulse = None
    servo_rear_leg_left_max_pulse = None
    servo_rear_leg_left_rest_angle = None

    servo_rear_feet_left = None
    servo_rear_feet_left_pca9685 = None
    servo_rear_feet_left_channel = None
    servo_rear_feet_left_min_pulse = None
    servo_rear_feet_left_max_pulse = None
    servo_rear_feet_left_rest_angle = None

    servo_rear_shoulder_right = None
    servo_rear_shoulder_right_pca9685 = None
    servo_rear_shoulder_right_channel = None
    servo_rear_shoulder_right_min_pulse = None
    servo_rear_shoulder_right_max_pulse = None
    servo_rear_shoulder_right_rest_angle = None

    servo_rear_leg_right = None
    servo_rear_leg_right_pca9685 = None
    servo_rear_leg_right_channel = None
    servo_rear_leg_right_min_pulse = None
    servo_rear_leg_right_max_pulse = None
    servo_rear_leg_right_rest_angle = None

    servo_rear_feet_right = None
    servo_rear_feet_right_pca9685 = None
    servo_rear_feet_right_channel = None
    servo_rear_feet_right_min_pulse = None
    servo_rear_feet_right_max_pulse = None
    servo_rear_feet_right_rest_angle = None

    servo_front_shoulder_left = None
    servo_front_shoulder_left_pca9685 = None
    servo_front_shoulder_left_channel = None
    servo_front_shoulder_left_min_pulse = None
    servo_front_shoulder_left_max_pulse = None
    servo_front_shoulder_left_rest_angle = None

    servo_front_leg_left = None
    servo_front_leg_left_pca9685 = None
    servo_front_leg_left_channel = None
    servo_front_leg_left_min_pulse = None
    servo_front_leg_left_max_pulse = None
    servo_front_leg_left_rest_angle = None

    servo_front_feet_left = None
    servo_front_feet_left_pca9685 = None
    servo_front_feet_left_channel = None
    servo_front_feet_left_min_pulse = None
    servo_front_feet_left_max_pulse = None
    servo_front_feet_left_rest_angle = None

    servo_front_shoulder_right = None
    servo_front_shoulder_right_pca9685 = None
    servo_front_shoulder_right_channel = None
    servo_front_shoulder_right_min_pulse = None
    servo_front_shoulder_right_max_pulse = None
    servo_front_shoulder_right_rest_angle = None

    servo_front_leg_right = None
    servo_front_leg_right_pca9685 = None
    servo_front_leg_right_channel = None
    servo_front_leg_right_min_pulse = None
    servo_front_leg_right_max_pulse = None
    servo_front_leg_right_rest_angle = None

    servo_front_feet_right = None
    servo_front_feet_right_pca9685 = None
    servo_front_feet_right_channel = None
    servo_front_feet_right_min_pulse = None
    servo_front_feet_right_max_pulse = None
    servo_front_feet_right_rest_angle = None

    servo_arm_rotation = None
    servo_arm_rotation_pca9685 = None
    servo_arm_rotation_channel = None
    servo_arm_rotation_min_pulse = None
    servo_arm_rotation_max_pulse = None
    servo_arm_rotation_rest_angle = None

    servo_arm_lift = None
    servo_arm_lift_pca9685 = None
    servo_arm_lift_channel = None
    servo_arm_lift_min_pulse = None
    servo_arm_lift_max_pulse = None
    servo_arm_lift_rest_angle = None

    servo_arm_range = None
    servo_arm_range_pca9685 = None
    servo_arm_range_channel = None
    servo_arm_range_min_pulse = None
    servo_arm_range_max_pulse = None
    servo_arm_range_rest_angle = None

    servo_arm_cam_tilt = None
    servo_arm_cam_tilt_pca9685 = None
    servo_arm_cam_tilt_channel = None
    servo_arm_cam_tilt_min_pulse = None
    servo_arm_cam_tilt_max_pulse = None
    servo_arm_cam_tilt_rest_angle = None

    myservo = None

    # pre assign array to hold up to two pca9685 cards.
    pca9685 = [{ "cardid": False, "address": None, "reference_clock_freq": None,	"frequency": None, "object": False },
               { "cardid": False, "address": None, "reference_clock_freq": None,	"frequency": None, "object": False }]

    servo_list = [ 'RLS', 'RLL', 'RLF',
                   'RRS', 'RRL', 'RRF',
                   'FLS', 'FLL', 'FLF',
                   'FRS', 'FRL', 'FRF' ]

    def __init__(self, communication_queues):

        try:

            log.debug('Starting controller...')

            signal.signal(signal.SIGINT, self.exit_gracefully)
            signal.signal(signal.SIGTERM, self.exit_gracefully)

            self._abort_queue = communication_queues[queues.ABORT_CONTROLLER]
            self._lcd_screen_queue = communication_queues[queues.LCD_SCREEN_CONTROLLER]

            # The boards are not initalized yet so display that fact.
            self._lcd_screen_queue.put('motion_controller_1 NOK')
            self._lcd_screen_queue.put('motion_controller_2 NOK')

            self.i2c = busio.I2C(SCL, SDA)
            self.load_pca9685_boards_configuration()

            self._motion_queue = communication_queues[queues.MOTION_CONTROLLER]

            self._previous_event = {}

        except Exception as e:
            log.error('Motion controller initialization problem', e)
            self._lcd_screen_queue.put('motion_controller_1 NOK')
            self._lcd_screen_queue.put('motion_controller_2 NOK')
            try:
                self.pca9685[0]['object'].deinit()
            finally:
                try:
                    if self.boards == 2:
                        self.pca9685[1]['object'].deinit()
                finally:
                    sys.exit(1)

    def exit_gracefully(self, signum, frame):
        try:
            self.pca9685[0]['object'].deinit()
        finally:
            try:
                if self.boards == 2:
                    self.pca9685[1]['object'].deinit()
            finally:
                log.info('Terminated')
                sys.exit(0)

    def do_process_events_from_queues(self):

        while True:

            try:

                event = self._motion_queue.get(block=True, timeout=60)

                # log.debug(event)

                if event['start']:
                    if self.is_activated:
                        self.rest_position()
                        time.sleep(0.5)
                        self.deactivate_pca9685_boards()
                        self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                    else:
                        self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ACTIVATE)
                        self.activate_pca9685_boards()
                        self.create_servos()
                        self.rest_position()


                if not self.is_activated:
                    log.info('Press START/OPTIONS to enable the servos')
                    continue

                if event['a']:
                    self.rest_position()

                if event['hat0y']:
                    self.body_move_body_up_and_down(event['hat0y'])

                if event['hat0x']:
                    self.body_move_body_left_right(event['hat0x'])

                if event['ry']:
                    self.body_move_body_up_and_down_analog(event['ry'])

                if event['rx']:
                    self.body_move_body_left_right_analog(event['rx'])

                if event['hat0x'] and event['tl2']:
                    # 2 buttons example
                    pass

                if event['y']:
                    self.standing_position()

                if event['b']:
                    self.body_move_position_right()

                if event['x']:
                    self.body_move_position_left()



            except queue.Empty as e:
                log.info('Inactivity lasted 60 seconds, shutting down the servos, '
                         'press start to reactivate')
                if self.is_activated:
                    self.rest_position()
                    time.sleep(0.5)
                    self.deactivate_pca9685_boards()

            except Exception as e:
                log.error('Unknown problem while processing the queue of the motion controller')
                log.error(' - Most likely a servo is not able to get to the assigned position')

    #############################
    # Load the PCA9685 boards Config
    def load_pca9685_boards_configuration(self):
        self.pca9685[0]["cardid"] = 1
        self.pca9685[0]["address"] = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_ADDRESS), 0)
        self.pca9685[0]["reference_clock_freq"] = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_REFERENCE_CLOCK_SPEED))
        self.pca9685[0]["frequency"] = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_FREQUENCY))

        self.pca9685[1]["address"] = False
        try:
            self.pca9685[1]["address"] = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_ADDRESS), 0)

            if self.pca9685[1]["address"]:
                self.pca9685[1]["cardid"] = 2
                self.pca9685[1]["reference_clock_freq"] = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_REFERENCE_CLOCK_SPEED))
                self.pca9685[1]["frequency"] = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_FREQUENCY))

        except Exception as e:
            log.debug("Only 1 PCA9685 is present in the configuration")

    #############################
    # Activate the PCA9685 boards
    def activate_pca9685_boards(self):

        for x in self.pca9685:
            if not isinstance(x['object'], PCA9685):
                x['object'] = PCA9685(self.i2c, address=x["address"], reference_clock_speed=x["reference_clock_freq"])
                x['object'].frequency = x["frequency"]
                mgs = 'motion_controller_' + str(cardid) + ' OK'
                self._lcd_screen_queue.put(msg)

            self.boards = len(self.pca9685)

        self.is_activated = True
        log.debug(str(self.boards) + ' PCA9685 board(s) activated')

    #############################
    # Deactivate the PCA9685 boards
    def deactivate_pca9685_boards(self):

        for x in self.pca9685:
            if isinstance(x['object'], PCA9685):
                x['object'].deinit()
                mgs = 'motion_controller_' + str(cardid) + ' NOK'
                self._lcd_screen_queue.put(msg)
                x['object'] = False

        self.is_activated = False
        log.debug(str(self.boards) + ' PCA9685 board(s) deactivated')


    #############################
    # Create the servo instances.
    def create_servos(self):

        # Rear Left Sholder
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685) == 1:
            self.myservo["RLS"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MAX_PULSE))
        else:
            self.myservo["RLS"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MAX_PULSE))
        self.myservo["RLS"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)

        # Rear Left Leg
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685) == 1:
            self.myservo["RLL"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MAX_PULSE))
        else:
            self.myservo["RLL"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MAX_PULSE))
        self.myservo["RLL"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)

        # Rear Left Foot
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_PCA9685) == 1:
            self.myservo["RLF"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_MAX_PULSE))
        else:
            self.myservo["RLF"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_MAX_PULSE))
        self.myservo["RLF"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_LEFT_REST_ANGLE)

        # Rear Right Sholder
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685) == 1:
            self.myservo["RRS"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MAX_PULSE))
        else:
            self.myservo["RRS"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MAX_PULSE))
        self.myservo["RRS"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)

        # Rear Right Leg
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685) == 1:
            self.myservo["RRL"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MAX_PULSE))
        else:
            self.myservo["RRL"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MAX_PULSE))
        self.myservo["RRL"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)

        # Rear Right Foot
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_PCA9685) == 1:
            self.myservo["RRF"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_MAX_PULSE))
        else:
            self.myservo["RRF"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_MAX_PULSE))
        self.myservo["RRF"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FOOT_RIGHT_REST_ANGLE)

        # Front Left Sholder
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685) == 1:
            self.myservo["FLS"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MAX_PULSE))
        else:
            self.myservo["FLS"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MAX_PULSE))
        self.myservo["FLS"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)

        # Front Left Leg
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685) == 1:
            self.myservo["FLL"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MAX_PULSE))
        else:
            self.myservo["FLL"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MAX_PULSE))
        self.myservo["FLL"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)

        # Front Left Foot
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_PCA9685) == 1:
            self.myservo["FLF"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_MAX_PULSE))
        else:
            self.myservo["FLF"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_MAX_PULSE))
        myservo["FLF"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_LEFT_REST_ANGLE)

        # Front Right Sholder
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685) == 1:
            self.myservo["FRS"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MAX_PULSE))
        else:
            self.myservo["FRS"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MAX_PULSE))
        self.myservo["FRS"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)

        # Front Right Leg
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685) == 1:
            self.myservo["FRL"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MAX_PULSE))
        else:
            self.myservo["FRL"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MAX_PULSE))
        self.myservo["FRL"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)

        # Front Right Foot
        if Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_PCA9685) == 1:
            self.myservo["FRF"] = MyServo(self.pca9685[0]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_MAX_PULSE))
        else:
            self.myservo["FRF"] = MyServo(self.pca9685[1]['object'].channels[Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_PCA9685)],
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_MIN_PULSE),
                                                                        Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_MAX_PULSE))
        self.myservo["FRF"].rest = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FOOT_RIGHT_REST_ANGLE)



    def activate_servos(self):

        if self.servo_rear_shoulder_left_pca9685 == 1:
            self.servo_rear_shoulder_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_shoulder_left_channel])
        else:
            self.servo_rear_shoulder_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_shoulder_left_channel])
        self.servo_rear_shoulder_left.set_pulse_width_range(min_pulse=self.servo_rear_shoulder_left_min_pulse, max_pulse=self.servo_rear_shoulder_left_max_pulse)

        if self.servo_rear_leg_left_pca9685 == 1:
            self.servo_rear_leg_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_leg_left_channel])
        else:
            self.servo_rear_leg_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_leg_left_channel])
        self.servo_rear_leg_left.set_pulse_width_range(min_pulse=self.servo_rear_leg_left_min_pulse, max_pulse=self.servo_rear_leg_left_max_pulse)

        if self.servo_rear_feet_left_pca9685 == 1:
            self.servo_rear_feet_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_feet_left_channel])
        else:
            self.servo_rear_feet_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_feet_left_channel])
        self.servo_rear_feet_left.set_pulse_width_range(min_pulse=self.servo_rear_feet_left_min_pulse, max_pulse=self.servo_rear_feet_left_max_pulse)

        if self.servo_rear_shoulder_right_pca9685 == 1:
            self.servo_rear_shoulder_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_shoulder_right_channel])
        else:
            self.servo_rear_shoulder_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_shoulder_right_channel])
        self.servo_rear_shoulder_right.set_pulse_width_range(min_pulse=self.servo_rear_shoulder_right_min_pulse, max_pulse=self.servo_rear_shoulder_right_max_pulse)

        if self.servo_rear_leg_right_pca9685 == 1:
            self.servo_rear_leg_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_leg_right_channel])
        else:
            self.servo_rear_leg_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_leg_right_channel])
        self.servo_rear_leg_right.set_pulse_width_range(min_pulse=self.servo_rear_leg_right_min_pulse, max_pulse=self.servo_rear_leg_right_max_pulse)

        if self.servo_rear_feet_right_pca9685 == 1:
            self.servo_rear_feet_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_feet_right_channel])
        else:
            self.servo_rear_feet_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_feet_right_channel])
        self.servo_rear_feet_right.set_pulse_width_range(min_pulse=self.servo_rear_feet_right_min_pulse, max_pulse=self.servo_rear_feet_right_max_pulse)

        if self.servo_front_shoulder_left_pca9685 == 1:
            self.servo_front_shoulder_left = servo.Servo(self.pca9685_1.channels[self.servo_front_shoulder_left_channel])
        else:
            self.servo_front_shoulder_left = servo.Servo(self.pca9685_2.channels[self.servo_front_shoulder_left_channel])
        self.servo_front_shoulder_left.set_pulse_width_range(min_pulse=self.servo_front_shoulder_left_min_pulse, max_pulse=self.servo_front_shoulder_left_max_pulse)

        if self.servo_front_leg_left_pca9685 == 1:
            self.servo_front_leg_left = servo.Servo(self.pca9685_1.channels[self.servo_front_leg_left_channel])
        else:
            self.servo_front_leg_left = servo.Servo(self.pca9685_2.channels[self.servo_front_leg_left_channel])
        self.servo_front_leg_left.set_pulse_width_range(min_pulse=self.servo_front_leg_left_min_pulse, max_pulse=self.servo_front_leg_left_max_pulse)

        if self.servo_front_feet_left_pca9685 == 1:
            self.servo_front_feet_left = servo.Servo(self.pca9685_1.channels[self.servo_front_feet_left_channel])
        else:
            self.servo_front_feet_left = servo.Servo(self.pca9685_2.channels[self.servo_front_feet_left_channel])
        self.servo_front_feet_left.set_pulse_width_range(min_pulse=self.servo_front_feet_left_min_pulse, max_pulse=self.servo_front_feet_left_max_pulse)

        if self.servo_front_shoulder_right_pca9685 == 1:
            self.servo_front_shoulder_right = servo.Servo(self.pca9685_1.channels[self.servo_front_shoulder_right_channel])
        else:
            self.servo_front_shoulder_right = servo.Servo(self.pca9685_2.channels[self.servo_front_shoulder_right_channel])
        self.servo_front_shoulder_right.set_pulse_width_range(min_pulse=self.servo_front_shoulder_right_min_pulse, max_pulse=self.servo_front_shoulder_right_max_pulse)

        if self.servo_front_leg_right_pca9685 == 1:
            self.servo_front_leg_right = servo.Servo(self.pca9685_1.channels[self.servo_front_leg_right_channel])
        else:
            self.servo_front_leg_right = servo.Servo(self.pca9685_2.channels[self.servo_front_leg_right_channel])
        self.servo_front_leg_right.set_pulse_width_range(min_pulse=self.servo_front_leg_right_min_pulse, max_pulse=self.servo_front_leg_right_max_pulse)

        if self.servo_front_feet_right_pca9685 == 1:
            self.servo_front_feet_right = servo.Servo(self.pca9685_1.channels[self.servo_front_feet_right_channel])
        else:
            self.servo_front_feet_right = servo.Servo(self.pca9685_2.channels[self.servo_front_feet_right_channel])
        self.servo_front_feet_right.set_pulse_width_range(min_pulse=self.servo_front_feet_right_min_pulse, max_pulse=self.servo_front_feet_right_max_pulse)


    def move(self):

        for x in servo_list:
            try:
                log.info('Moving servo ' + x + ' to angle ' + str(self.myservo[x].newangle))
                self.myservo[x].move()
            except ValueError as e:
                log.error('Error moving servo ' + x + '. Impossable angle requested')


    def rest_position(self):

        for x in servo_list:
            self.myservo[x].newangle = self.myservo[x].rest
            self.move()

    def body_move_body_up_and_down(self, raw_value):

        range = 10
        range2 = 15

        # TODO: Need to add checks so it can't go beyond rest or stand for the ranges.
        if raw_value < 0:
            self.myservo['RLL'].newangle = self.myservo['RLL'].currentangle - range
            self.myservo['RLF'].newangle = self.myservo['RLF'].currentangle + range2
            self.myservo['RRL'].newangle = self.myservo['RRL'].currentangle + range
            self.myservo['RLF'].newangle = self.myservo['RLF'].currentangle - range2
            self.myservo['FLL'].newangle = self.myservo['FLL'].currentangle - range
            self.myservo['FLF'].newangle = self.myservo['FLF'].currentangle + range2
            self.myservo['FRL'].newangle = self.myservo['FRL'].currentangle + range
            self.myservo['FRF'].newangle = self.myservo['FRF'].currentangle - range2

        elif raw_value > 0:
            self.myservo['RLL'].newangle = self.myservo['RLL'].currentangle - range
            self.myservo['RLF'].newangle = self.myservo['RLF'].currentangle + range2
            self.myservo['RRL'].newangle = self.myservo['RRL'].currentangle + range
            self.myservo['RLF'].newangle = self.myservo['RLF'].currentangle - range2
            self.myservo['FLL'].newangle = self.myservo['FLL'].currentangle - range
            self.myservo['FLF'].newangle = self.myservo['FLF'].currentangle + range2
            self.myservo['FRL'].newangle = self.myservo['FRL'].currentangle + range
            self.myservo['FRF'].newangle = self.myservo['FRF'].currentangle - range2
        else:
            self.rest_position()

        self.move()

    def body_move_body_up_and_down_analog(self, raw_value):

        servo_rear_leg_left_max_angle = 38
        servo_rear_feet_left_max_angle = 70
        servo_rear_leg_right_max_angle = 126
        servo_rear_feet_right_max_angle = 102
        servo_front_leg_left_max_angle = 57
        servo_front_feet_left_max_angle = 85
        servo_front_leg_right_max_angle = 130
        servo_front_feet_right_max_angle = 120

        delta_rear_leg_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE), servo_rear_leg_left_max_angle), raw_value))
        delta_rear_feet_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE), servo_rear_feet_left_max_angle), raw_value))
        delta_rear_leg_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE), servo_rear_leg_right_max_angle), raw_value))
        delta_rear_feet_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE), servo_rear_feet_right_max_angle), raw_value))
        delta_front_leg_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE), servo_front_leg_left_max_angle), raw_value))
        delta_front_feet_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE), servo_front_feet_left_max_angle), raw_value))
        delta_front_leg_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE), servo_front_leg_right_max_angle), raw_value))
        delta_front_feet_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE), servo_front_feet_right_max_angle), raw_value))

        self.myservo['RLL'].newangle = delta_rear_leg_left
        self.myservo['RLF'].newangle = delta_rear_feet_left
        self.myservo['RRL'].newangle = delta_rear_leg_right
        self.myservo['RRF'].newangle = delta_rear_feet_right
        self.myservo['FLL'].newangle = delta_front_leg_left
        self.myservo['FLF'].newangle = delta_front_feet_left
        self.myservo['FRL'].newangle = delta_front_leg_right
        self.myservo['FRF'].newangle = delta_front_feet_right

        self.move()

    def body_move_body_left_right(self, raw_value):

        range = 5

        if raw_value < 0:
            self.myservo['RLS'].newangle = self.myservo['RLS'].currentangle - range
            self.myservo['RRS'].newangle = self.myservo['RRS'].currentangle - range
            self.myservo['FLS'].newangle = self.myservo['FLS'].currentangle + range
            self.myservo['FRS'].newangle = self.myservo['FRS'].currentangle + range

        elif raw_value > 0:
            self.myservo['RLS'].newangle = self.myservo['RLS'].currentangle + range
            self.myservo['RRS'].newangle = self.myservo['RRS'].currentangle + range
            self.myservo['FLS'].newangle = self.myservo['FLS'].currentangle - range
            self.myservo['FRS'].newangle = self.myservo['FRS'].currentangle - range

        else:
            self.rest_position()

        self.move()

    def body_move_body_left_right_analog(self, raw_value):

        delta_a = int(General().maprange((-1, 1), (30, 150), raw_value))
        delta_b = int(General().maprange((-1, 1), (150, 30), raw_value))

        self.myservo['RLS'].newangle = delta_a
        self.myservo['RRS'].newangle = delta_a
        self.myservo['FLS'].newangle = delta_b
        self.myservo['FRS'].newangle = delta_b

        self.move()

    def standing_position(self):

        variation_leg = 50
        variation_feet = 70

        self.myservo['RLS'].newangle = self.myservo['RLS'].rest + 10
        self.myservo['RRS'].newangle = self.myservo['RRS'].rest - 10
        self.myservo['FLS'].newangle = self.myservo['FLS'].rest + 10
        self.myservo['FRS'].newangle = self.myservo['FRS'].rest - 10

        self.myservo['RLL'].newangle = self.myservo['RLL'].rest - variation_leg
        self.myservo['RRL'].newangle = self.myservo['RRL'].rest + variation_leg
        self.myservo['FLL'].newangle = self.myservo['FLL'].rest - variation_leg + 5
        self.myservo['FRL'].newangle = self.myservo['FRL'].rest + variation_leg - 5

        self.myservo['RLF'].newangle = self.myservo['RLF'].rest + variation_feet
        self.myservo['RRF'].newangle = self.myservo['RRF'].rest - variation_feet
        self.myservo['FLF'].newangle = self.myservo['FLF'].rest + variation_feet - 5
        self.myservo['FRF'].newangle = self.myservo['FRF'].rest - variation_feet + 5

        self.move()

    def body_move_position_right(self):

        move = 20
        variation_leg = 50
        variation_feet = 70

        self.myservo['RLS'].newangle = self.myservo['RLS'].rest + 10 + move
        self.myservo['RLL'].newangle = self.myservo['RLL'].rest - variation_leg
        self.myservo['RLF'].newangle = self.myservo['RLF'].rest + variation_feet
        self.myservo['RRS'].newangle = self.myservo['RRS'].rest - 10 + move
        self.myservo['RRL'].newangle = self.myservo['RRL'].rest + variation_leg
        self.myservo['RRF'].newangle = self.myservo['RRF'].rest - variation_feet

        self.myservo['FLS'].newangle = self.myservo['FLS'].rest - 10 - move
        self.myservo['FLL'].newangle = self.myservo['FLL'].rest + variation_leg + 5
        self.myservo['FLF'].newangle = self.myservo['FLF'].rest - variation_feet - 5
        self.myservo['FRS'].newangle = self.myservo['FRS'].rest + 10 - move
        self.myservo['FRL'].newangle = self.myservo['FRL'].rest + variation_leg - 5
        self.myservo['FRF'].newangle = self.myservo['FRF'].rest - variation_feet + 5

        self.move()

    def body_move_position_left(self):

        move = 20

        variation_leg = 50
        variation_feet = 70

        self.myservo['RLS'].newangle = self.myservo['RLS'].rest + 10 - move
        self.myservo['RLL'].newangle = self.myservo['RLL'].rest - variation_leg
        self.myservo['RLF'].newangle = self.myservo['RLF'].rest + variation_feet
        self.myservo['RRS'].newangle = self.myservo['RRS'].rest - 10 - move
        self.myservo['RRL'].newangle = self.myservo['RRL'].rest + variation_leg
        self.myservo['RRF'].newangle = self.myservo['RRF'].rest - variation_feet

        self.myservo['FLS'].newangle = self.myservo['FLS'].rest - 10 + move
        self.myservo['FLL'].newangle = self.myservo['FLL'].rest + variation_leg + 5
        self.myservo['FLF'].newangle = self.myservo['FLF'].rest - variation_feet - 5
        self.myservo['FRS'].newangle = self.myservo['FRS'].rest + 10 + move
        self.myservo['FRL'].newangle = self.myservo['FRL'].rest + variation_leg - 5
        self.myservo['FRF'].newangle = self.myservo['FRF'].rest - variation_feet + 5

        self.move()


