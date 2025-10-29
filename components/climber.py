from enum import Enum

from phoenix6 import configs, controls, hardware, signals

# import utilities.constants as constants
# TODO import subsystem drivesb


class Climber:
    WINCH_REVS_PER_SHAFT_REV = 1.0 / (3 * 3 * 3)
    METERS_PER_WINCH_REV = 0.081118  # TODO: FIND ACTUAL
    # TODO: Find actual (height from ground to bottom of hooks when fully retracted)
    RETRACTED_METERS = 0.52
    MAX_HEIGHT_FROM_GROUND = 1.2
    TOLERANCE_METERS = 0.02

    MOTOR_ID = 10

    instance = None

    def __init__(self):
        # self.tab = Shuffleboard.getTab("Climber")

        self.motor = hardware.TalonFX(Climber.MOTOR_ID, "*")

        self.motor_config = configs.TalonFXConfiguration()
        self.motor_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # self.motor_config.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        self.motor_config.current_limits.supply_current_limit = 60
        self.motor.configurator.apply(self.motor_config)

        self._state = ClimberPositions.DISABLED
        self.previous_state = ClimberPositions.DISABLED

        # self.climbing_PID_controller = controller.ProfiledPIDController(600,0,0, trajectory.TrapezoidProfile.Constraints(9999999,3))
        # self.climbing_PID_controller.setTolerance(self.TOLERANCE_METERS)

        # self.climbing_feed_forward = controller.ElevatorFeedforward(0,0,0)

        self.manual_voltage = 0
        self.position_voltage = 0

        self.motor.set_position(0)
        Climber.instance = self

        # self.climbing_PID_controller.setGoal(self.get_height_from_ground())

    # def zero_motor(self, heightFromRetracted):
    #     self.motor.set_position(heightFromRetracted / self.METERS_PER_WINCH_REV / self.WINCH_REVS_PER_SHAFT_REV)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, desired_state):
        self._state = desired_state

    def get_height_from_ground(self):
        return self.get_height_from_retracted() + self.RETRACTED_METERS

    def get_height_from_retracted(self):
        return (
            self.motor.get_position().value
            * self.WINCH_REVS_PER_SHAFT_REV
            * self.METERS_PER_WINCH_REV
        )  # TODO REVIEW THIS

    def set_manual_voltage(self, voltage):
        if self.state != ClimberPositions.MANUAL:
            self.previous_state = self.state

        self.manual_voltage = voltage
        self.state = ClimberPositions.MANUAL

    def exit_manual(self):
        # self.climbing_PID_controller.reset(self.get_height_from_ground())
        self.state = self.previous_state

    def enable_position_control(self, position):
        # self.climbing_PID_controller.reset(self.get_height_from_ground())
        # self.climbing_PID_controller.setGoal(utility_functions.clamp(position, self.RETRACTED_METERS, self.MAX_HEIGHT_FROM_GROUND))

        # self.position_voltage = self.climbing_feed_forward.calculate(self.climbing_PID_controller.getSetpoint().velocity + self.climbing_PID_controller.calculate(self.get_height_from_ground()))
        self.state = ClimberPositions.POSITION_CONTROL

    def execute(self):
        match self.state:
            case ClimberPositions.POSITION_CONTROL:
                self.motor.set_control(controls.VoltageOut(self.position_voltage))

            case ClimberPositions.MANUAL:
                self.motor.set_control(controls.VoltageOut(self.manual_voltage))

            case ClimberPositions.DISABLED:
                self.motor.set_control(controls.VoltageOut(0))


class ClimberPositions(Enum):
    POSITION_CONTROL = 1
    MANUAL = 2
    DISABLED = 3
