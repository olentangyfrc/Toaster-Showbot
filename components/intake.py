import math

import wpilib
import wpimath.controller as controller
import wpimath.trajectory as trajectory
from phoenix6 import configs, controls, hardware, signals
from wpilib import DigitalInput

INTAKE_GEAR_RATIO = (1 / 5) * (1 / 3) * (16 / 34)

IDLE_POSITION = 94 * math.pi / 180
ZERO_POSITION = 96.843 * math.pi / 180
DEPLOYED_POSITION = -16.8 * math.pi / 180
FEEDING_POSITION = 90 * math.pi / 180

FEED_SPEED = 0.25 * 12
INTAKE_SPEED = 0.5 * 12

FEED_DELAY = 0.03
INTAKE_DELAY = 0.0

POSSIBLE_STATES = ["IDLE", "DEPLOYED", "RETRACTING", "MANUAL", "FEEDING", "EJECT"]


class Intake:
    instance = None

    def __init__(self) -> None:
        self._state = "IDLE"
        self.target_pivot_position = IDLE_POSITION
        self.target_roller_voltage = 0

        self.intake_rollers_motor = hardware.TalonFX(constants.INTAKE_ROLLERS_CAN, "*")
        self.beam_break = DigitalInput(constants.INTAKE_BEAM_BREAK)

        self.pivot_configs = configs.TalonFXConfiguration()
        self.pivot_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.pivot_configs.current_limits.supply_current_limit = 2
        self.pivot_configs.motor_output.inverted = (
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        self.pivot_motor = hardware.TalonFX(constants.INTAKE_PIVOT_CAN, "*")
        self.pivot_motor.configurator.apply(self.pivot_configs)

        self.pid_constraints = trajectory.TrapezoidProfile.Constraints(
            constants.INTAKE_PIVOT_MAX_VELOCITY, constants.INTAKE_PIVOT_MAX_ACCELERATION
        )
        self.pivot_ff = controller.ArmFeedforward(0, 0.45, 0)
        self.pivot_pid = controller.ProfiledPIDController(
            30, 0, 0.2, self.pid_constraints
        )
        self.pivot_pid.setTolerance(0.03)
        self.pivot_pid.reset(ZERO_POSITION)
        self.pivot_motor.set_position(ZERO_POSITION / math.tau / INTAKE_GEAR_RATIO)
        self.target_pivot_position = IDLE_POSITION
        self.intake_timer = wpilib.Timer()
        self.trigger_time = float("nan")

        self.pivot_pid.setGoal(self.target_pivot_position)

        Intake.instance = self

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        if new_state in POSSIBLE_STATES:
            self._state = new_state
        else:
            self._state = "IDLE"

    def go_to_idle(self):
        self.state = "IDLE"

    def force_feed(self):
        self.state = "FEEDING"

    def eject(self):
        self.state = "EJECT"

    def get_position_degrees(self):
        return self.pivot_motor.get_position().value * 360 * INTAKE_GEAR_RATIO

    def manual_move(self, radians) -> None:
        self.state = "MANUAL"
        self.target_pivot_position = radians

    def get_position_radians(self):
        return self.pivot_motor.get_position().value * math.tau * INTAKE_GEAR_RATIO

    def has_note(self) -> bool:
        return self.beam_break.get()

    def note_in_shooter(self) -> bool:
        if shooter.Shooter.instance is not None:
            return shooter.Shooter.instance.has_note()

    def grab_note(self):
        self.state = "DEPLOYED"

    @property
    def target(self):
        return self._target

    def pid_voltage(self):
        return self.pivot_pid.calculate(self.get_position_radians())

    def intake_pid_goal(self):
        return self.pivot_pid.getGoal().position

    def ff_voltage(self):
        return self.pivot_ff.calculate(self.get_position_radians(), 0)

    @target.setter
    def target(self, target_val):
        self._target = target_val

    def at_target_position(self) -> None:
        return self.pivot_pid.atGoal()

    def execute(self):
        match self.state:
            case "MANUAL":
                self.target_roller_voltage = 0

            case "IDLE":
                self.target_pivot_position = IDLE_POSITION
                self.target_roller_voltage = 0

            case "EJECT":
                self.target_pivot_position = math.pi / 4
                self.target_roller_voltage = 4

            case "DEPLOYED":
                if math.isnan(self.trigger_time):
                    self.target_pivot_position = DEPLOYED_POSITION
                    self.target_roller_voltage = INTAKE_SPEED
                    if self.has_note():
                        self.trigger_time = self.intake_timer.getFPGATimestamp()
                elif (
                    self.intake_timer.getFPGATimestamp() - self.trigger_time
                    <= INTAKE_DELAY
                ):
                    self.target_pivot_position = DEPLOYED_POSITION
                    self.target_roller_voltage = INTAKE_SPEED
                else:
                    self.target_pivot_position = FEEDING_POSITION
                    self.target_roller_voltage = 0
                    self.state = "RETRACTING"
                    self.trigger_time = float("nan")

            case "RETRACTING":
                self.target_pivot_position = FEEDING_POSITION
                self.target_roller_voltage = 0
                if shooter.Shooter.instance is not None:
                    shooter.Shooter.instance.feed()
                if self.at_target_position():
                    self.state = "FEEDING"

            case "FEEDING":
                self.target_pivot_position = FEEDING_POSITION
                self.target_roller_voltage = FEED_SPEED
                if (not self.has_note()) and self.note_in_shooter():
                    self.state = "IDLE"

        self.target_pivot_position = utils.clamp(
            self.target_pivot_position, DEPLOYED_POSITION, ZERO_POSITION
        )
        self.pivot_pid.setGoal(self.target_pivot_position)

        self.intake_rollers_motor.set_control(
            controls.VoltageOut(self.target_roller_voltage)
        )
        self.pivot_motor.set_control(
            controls.VoltageOut(
                self.pid_voltage()
                + self.pivot_ff.calculate(self.get_position_radians(), 0)
            )
        )
