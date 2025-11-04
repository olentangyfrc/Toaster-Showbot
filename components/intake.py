import math
from enum import Enum

import wpilib
import wpimath.controller as controller
import wpimath.trajectory as trajectory
import wpimath.units as units
from magicbot import tunable
from phoenix6 import BaseStatusSignal, configs, controls, hardware, signals
from wpilib import Color8Bit, DigitalInput, MechanismRoot2d, RobotBase, SmartDashboard
from wpimath.filter import SlewRateLimiter

from utilities.configs import IntakeConfig
from utilities.IO import IntakeIO

# TODO:
#   create a config with CAN ID's, Gear Ratio, PID Constants
#   create IO class

ZERO_POSITION = math.radians(96.843)  # Prolly should dbl check this number

FEED_DELAY = 0.03
INTAKE_DELAY = 0.03


class IntakeStates(Enum):
    IDLE = 1
    DEPLOYED = 2
    RETRACTING = 3
    FEEDING = 5
    EJECT = 6


class Intake:
    manual_tuning_mode = tunable(False)

    def __init__(self, config: IntakeConfig, mech_root: MechanismRoot2d) -> None:
        self.intake_rollers_motor = hardware.TalonFX(config.roller_id, config.CANbus)
        self.beam_break = DigitalInput(config.beam_break_id)

        self.pivot_configs = configs.TalonFXConfiguration()
        self.pivot_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # self.pivot_configs.current_limits.supply_current_limit = 2  # What???
        self.pivot_configs.motor_output.inverted = (
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        self.pivot_motor = hardware.TalonFX(config.pivot_motor_id, config.CANbus)
        self.pivot_motor.configurator.apply(self.pivot_configs)

        self.pid_constraints = trajectory.TrapezoidProfile.Constraints(
            config.pivot_max_vel, config.pivot_max_acc
        )
        self.pivot_ff = controller.ArmFeedforward(
            config.pivot_ff.kS,
            config.pivot_ff.kG,  # type: ignore
            config.pivot_ff.kV,
            config.pivot_ff.kA,
        )
        self.pivot_pid = controller.ProfiledPIDController(
            config.pivot_pid.p,
            config.pivot_pid.i,
            config.pivot_pid.d,
            self.pid_constraints,
        )
        self.pivot_pid.setTolerance(0.03)
        self.pivot_pid.reset(ZERO_POSITION)
        self.pivot_motor.set_position(ZERO_POSITION / math.tau / self.config.gear_ratio)

        self._state = IntakeStates.IDLE

        self.intake_voltage_request = controls.VoltageOut(0)
        self.pivot_voltage_request = controls.VoltageOut(0)

        self.intake_timer = wpilib.Timer()

        self.config = config

        self.io = IntakeIO()
        self._set_up_logging()

        # TODO: Replace this approach with actual simulation
        self.sim_angle = ZERO_POSITION
        self.sim_limiter = SlewRateLimiter(1.5)

        self.mech_ligament = mech_root.appendLigament(
            name="Arm ligament",
            length=1.5,
            angle=math.degrees(self.sim_angle),
            color=Color8Bit(0, 0, 255),  # RGB
        )

    @property
    def state(self) -> IntakeStates:
        return self._state

    @state.setter
    def state(self, state: IntakeStates) -> None:
        if not isinstance(state, IntakeStates):
            raise ValueError("State must be of type IntakeStates")
        self._state = state

    def go_to_idle(self) -> None:
        self.state = IntakeStates.IDLE

    def force_feed(self) -> None:
        self.state = IntakeStates.FEEDING

    def eject(self) -> None:
        self.state = IntakeStates.EJECT

    def grab_note(self) -> None:
        self.state = IntakeStates.DEPLOYED

    def get_pivot_position(self) -> units.radians:
        if RobotBase.isSimulation():
            return self.sim_angle

        return (
            self.io.pivot_position_supplier.value * math.tau * self.config.gear_ratio
        )  # Change this to IO

    def get_pivot_velocity(self) -> units.radians_per_second:
        return self.io.pivot_velocity_supplier.value * math.tau * self.config.gear_ratio

    def has_note(self) -> bool:
        return self.beam_break.get()

    def at_target_position(self) -> bool:
        if RobotBase.isSimulation():
            return abs(self.sim_angle - self.pivot_pid.getGoal().position) <= self.pivot_pid.getPositionTolerance() 
        return self.pivot_pid.atGoal()

    def execute(self):
        self._handle_state_logic()

        if (
            self.io.target_pivot_position != self.pivot_pid.getGoal().position
            and not self.manual_tuning_mode
        ):
            self.pivot_pid.setGoal(self.io.target_pivot_position)

        if (
            RobotBase.isSimulation()
            and not self.at_target_position()
        ):
            if self.pivot_pid.getP() == 0:
                return
            
            intermediate = self.sim_angle

            intermediate += (self.pivot_pid.getGoal().position - self.sim_angle)/self.pivot_pid.getP()
            self.sim_angle = self.sim_limiter.calculate(intermediate)

            self.mech_ligament.setAngle(math.degrees(self.get_pivot_position()))
            return

        self.pivot_motor.set_control(
            self.pivot_voltage_request.with_output(
                self.pivot_pid.calculate(self.get_pivot_position())
                + self.pivot_ff.calculate(
                    self.get_pivot_position(), self.get_pivot_velocity()
                )
            ).with_enable_foc(False)
        )

        if self.at_target_position():
            self.intake_rollers_motor.set_control(
                self.intake_voltage_request.with_output(
                    self.io.target_roller_voltage
                ).with_enable_foc(False)
            )

    def _handle_state_logic(self) -> None:
        if self.manual_tuning_mode and not self.io.tuning_sendables_sent:
            SmartDashboard.putData("Intake Pivot PID", self.pivot_pid)
            self.io.tuning_sendables_sent = True

        self.io.state = self.state.name

        match self._state:
            case IntakeStates.IDLE:
                self.io.target_roller_voltage = 0
                self.io.target_pivot_position = math.radians(94)

            case IntakeStates.EJECT:
                self.io.target_roller_voltage = 4
                self.io.target_pivot_position = math.radians(45)

            case IntakeStates.DEPLOYED:
                self.io.target_roller_voltage = 6
                self.io.target_pivot_position = math.radians(-16.8)

                if self.has_note():
                    if not self.intake_timer.isRunning():
                        self.intake_timer.start()

                    if self.intake_timer.get() >= INTAKE_DELAY:
                        self.intake_timer.stop()
                        self.intake_timer.reset()

                        self.state = IntakeStates.IDLE

            case IntakeStates.FEEDING:
                self.io.target_roller_voltage = 4
                self.io.target_pivot_position = math.radians(90)

                if not self.has_note():
                    self.state = IntakeStates.IDLE

    def _set_up_logging(self) -> None:
        self.io.add_function(self.has_note)
        self.io.add_function(self.at_target_position)
        self.io.add_function(self.get_pivot_position)
        self.io.add_function(self.get_pivot_velocity)

        self.io.target_pivot_position = math.radians(94)
        self.io.target_roller_voltage = 0
        self.pivot_pid.setGoal(self.io.target_pivot_position)

        self.io.pivot_position_supplier = self.pivot_motor.get_position()
        self.io.pivot_velocity_supplier = self.pivot_motor.get_velocity()

        BaseStatusSignal.set_update_frequency_for_all(
            250, self.io.pivot_position_supplier, self.io.pivot_velocity_supplier
        )
