import math
from enum import Enum

import wpilib
import wpimath.units as units
from magicbot import tunable
from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpilib import Color8Bit, DigitalInput, MechanismRoot2d, RobotBase, SmartDashboard
from wpimath.controller import ArmFeedforward, ProfiledPIDController
from wpimath.filter import SlewRateLimiter
from wpimath.trajectory import TrapezoidProfile

from utilities.configs import IntakeConfig
from utilities.helpers import SimplePControllerSim, clamp
from utilities.IO import IntakeIO

ZERO_POSITION = math.radians(96.843)
MIN_POSITION = math.radians(-17)

FEED_DELAY = 0.03
INTAKE_DELAY = 0


class IntakeStates(Enum):
    IDLE = 1
    DEPLOYED = 2
    RETRACTING = 3
    FEEDING = 5
    EJECT = 6
    BOOT_UP = 7


class Intake:
    manual_tuning_mode = tunable(False)

    def __init__(self, config: IntakeConfig, mech_root: MechanismRoot2d) -> None:
        self.intake_rollers_motor = TalonFX(config.roller_id, config.CANbus)
        self.beam_break = DigitalInput(config.beam_break_id)
        self.mag_switch = DigitalInput(config.mag_switch_id)

        self.pivot_configs = TalonFXConfiguration()
        self.pivot_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.pivot_configs.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        self.pivot_motor = TalonFX(config.pivot_motor_id, config.CANbus)
        self.pivot_motor.configurator.apply(self.pivot_configs)

        self.pid_constraints = TrapezoidProfile.Constraints(
            config.profile_constants.max_vel, config.profile_constants.max_acc
        )
        self.pivot_ff = ArmFeedforward(
            config.pivot_ff.kS,
            config.pivot_ff.kG if config.pivot_ff.kG is not None else 0.0,
            config.pivot_ff.kA,
            config.pivot_ff.kV,
        )

        self.pivot_pid = ProfiledPIDController(
            config.pivot_pid.p,
            config.pivot_pid.i,
            config.pivot_pid.d,
            self.pid_constraints,
        )
        self.pivot_pid.setTolerance(0.03)
        self.pivot_pid.reset(ZERO_POSITION)

        self.boot_up_finished = False
        
        self._state = IntakeStates.BOOT_UP

        self.intake_voltage_request = VoltageOut(0)
        self.pivot_voltage_request = VoltageOut(0)

        self.intake_timer = wpilib.Timer()

        self.config = config

        self.pid_sim = SimplePControllerSim(
            self.pivot_pid, SlewRateLimiter(6), ZERO_POSITION
        )

        self.mech_ligament = mech_root.appendLigament(
            name="Arm ligament",
            length=1.5,
            angle=math.degrees(self.get_pivot_angle()),
            color=Color8Bit(0, 0, 255),  # RGB
        )

        self.io = IntakeIO()
        self._set_up_logging()

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

    def get_pivot_angle(self) -> units.radians:
        if RobotBase.isSimulation():
            return self.pid_sim.value

        return self.io.pivot_position_supplier.value * math.tau * self.config.gear_ratio

    def has_note(self) -> bool:
        return self.beam_break.get()

    def at_target_position(self) -> bool:
        if RobotBase.isSimulation():
            return self.pid_sim.at_goal()
        return self.pivot_pid.atGoal()
    
    def at_rest(self) -> bool: 
        self.mag_switch.get()

    def execute(self):
        self._handle_state_logic()

        self.pid_sim.update()

        if (
            self.io.target_pivot_angle != self.pivot_pid.getGoal().position
            and not self.manual_tuning_mode
        ):
            self.pivot_pid.setGoal(
                clamp(self.io.target_pivot_angle, MIN_POSITION, ZERO_POSITION)
            )
        elif self.manual_tuning_mode:
            self.pivot_pid.setGoal(
                clamp(self.pivot_pid.getGoal().position, MIN_POSITION, ZERO_POSITION)
            )

        if RobotBase.isSimulation():
            self.pid_sim.update()
            self.mech_ligament.setAngle(math.degrees(self.get_pivot_angle()))
            return

        self.io.pivot_voltage = self.pivot_pid.calculate(
            self.get_pivot_angle()
        ) + self.pivot_ff.calculate(self.get_pivot_angle(), 0)
        
        if not self.state == IntakeStates.BOOT_UP:
            self.pivot_motor.set_control(
                self.pivot_voltage_request.with_output(
                    self.io.pivot_voltage
                ).with_enable_foc(False)
            )
        else: 
            self.pivot_motor.set_control(
                self.pivot_voltage_request.with_output(
                    1
                ).with_enable_foc(False)
            )

        if self.at_target_position():
            self.intake_rollers_motor.set_control(
                self.intake_voltage_request.with_output(
                    self.io.roller_voltage
                ).with_enable_foc(False)
            )

    def _handle_state_logic(self) -> None:
        if self.manual_tuning_mode and not self.io.tuning_sendables_sent:
            SmartDashboard.putData("Intake Pivot PID", self.pivot_pid)
            self.io.tuning_sendables_sent = True

        self.io.state = self.state.name

        match self._state:
            case IntakeStates.BOOT_UP: 
                self.io.roller_voltage = 0

                if self.at_rest():
                    self.pivot_motor.set_position(ZERO_POSITION / math.tau / self.config.gear_ratio)
                    self.state = IntakeStates.IDLE
                    self.boot_up_finished = True

            case IntakeStates.IDLE:
                self.io.roller_voltage = 0
                self.io.target_pivot_angle = math.radians(94)

            case IntakeStates.EJECT:
                self.io.roller_voltage = -2  # Check pos and speed
                self.io.target_pivot_angle = math.radians(45)

                if not self.intake_timer.isRunning():
                    self.intake_timer.restart()
                if self.intake_timer.get() > 0.5:
                    self.state = IntakeStates.IDLE
                    self.intake_timer.stop()

            case IntakeStates.DEPLOYED:
                self.io.roller_voltage = 2
                self.io.target_pivot_angle = math.radians(-14)

                if self.has_note():
                    self.state = IntakeStates.RETRACTING

            case IntakeStates.FEEDING:
                self.io.roller_voltage = 2
                self.io.target_pivot_angle = math.radians(95.5)

            case IntakeStates.RETRACTING:
                self.io.roller_voltage = 0.2
                self.io.target_pivot_angle = math.radians(95.5)

                if self.at_target_position():
                    self.state = IntakeStates.FEEDING

    def _set_up_logging(self) -> None:
        self.io.add_function(self.has_note)
        self.io.add_function(self.at_target_position)
        self.io.add_function(self.get_pivot_angle, math.degrees)
        self.io.add_function(self.get_mag_switch)

        self.io.target_pivot_angle = math.radians(94)
        self.io.roller_voltage = 0
        self.pivot_pid.setGoal(self.io.target_pivot_angle)

        self.io.pivot_position_supplier = self.pivot_motor.get_position()

        BaseStatusSignal.set_update_frequency_for_all(
            250, self.io.pivot_position_supplier
        )
