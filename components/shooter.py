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
from rev import SparkBaseConfig, SparkMax, ResetMode, PersistMode
from wpilib import (
    Color8Bit,
    DigitalInput,
    DutyCycleEncoder,
    MechanismRoot2d,
    RobotBase,
    SmartDashboard,
    Timer,
)
from wpimath.controller import (
    ArmFeedforward,
    ProfiledPIDController,
    SimpleMotorFeedforwardRadians,
)
from wpimath.trajectory import TrapezoidProfile

from utilities.configs import ShooterConfig
from utilities.helpers import SimplePControllerSim, clamp
from utilities.IO import ShooterIO

FEED_DELAY = 0.06
FEEDING_ANGLE = math.radians(0)


INDEXER_FEED_VOLTAGE = -0.5 * 12
INDEXER_SHOOTING_VOLTAGE = -0.25 * 12
SHOOTER_SHOOTING_SPEED = 150


SUBWOOFER_SHOOTING_ANGLE = math.radians(40)
HOLDING_ANGLE = math.radians(25)

MIN_SHOOTER_ANGLE = math.radians(-7)
MAX_SHOOTER_ANGLE = math.radians(43)

MAX_SHOOTER_SPEED = 250 # Closer to 300 on actual robot bc bad ff tuning

ABS_ENCODER_OFFSET = 0.176  # Position absolute encoder reads when held horizontally


class ShooterStates(Enum):
    IDLE = 1
    AIMING = 2
    SHOOTING = 3
    FEEDING = 4
    HOLDING = 5
    EJECT = 6


class Shooter:
    manual_tuning_mode = tunable(False)
    manual_flywheel_speed = tunable(0.0)

    def __init__(self, config: ShooterConfig, mech_root: MechanismRoot2d):
        self.beam_break = DigitalInput(config.beam_break_id)
        self.angle_absolute_encoder = DutyCycleEncoder(config.pivot_abs_encoder_id)
        self.angle_absolute_encoder.setInverted(True)

        self.pivot_configs = TalonFXConfiguration()
        self.pivot_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # self.pivot_configs.current_limits.supply_current_limit = 2 #TODO: Decide if we want to uncomment this and rest of current limiters
        self.pivot_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.pivot_motor = TalonFX(config.pivot_motor_id, config.CANbus)
        self.pivot_motor.configurator.apply(self.pivot_configs)

        self.pivot_pid = ProfiledPIDController(
            config.pivot_pid.p,
            config.pivot_pid.i,
            config.pivot_pid.d,
            TrapezoidProfile.Constraints(
                config.pivot_profile_constraints.max_vel,
                config.pivot_profile_constraints.max_acc,
            ),
        )
        self.pivot_pid.setIZone(0.2)
        self.pivot_pid.setTolerance(0.02)

        self.pivot_ff = ArmFeedforward(
            config.pivot_ff.kS,
            config.pivot_ff.kG if config.pivot_ff.kG is not None else 0.0,
            config.pivot_ff.kA,
            config.pivot_ff.kV,
        )

        self.shoot_configs = TalonFXConfiguration()
        self.shoot_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.bottom_flywheel_motor = TalonFX(
            config.bottom_flywheel_motor_id, config.CANbus
        )
        self.bottom_flywheel_motor.configurator.apply(self.shoot_configs)

        self.top_flywheel_motor = TalonFX(config.top_flywheel_motor_id, config.CANbus)
        self.top_flywheel_motor.configurator.apply(self.shoot_configs)

        self.flywheel_ff = SimpleMotorFeedforwardRadians(
            config.shooter_speed_ff.kS,
            config.shooter_speed_ff.kV,
            config.shooter_speed_ff.kA,
        )

        self.indexer_config = (
            SparkBaseConfig()
            .inverted(False)
            .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        )

        self.indexer = SparkMax(config.indexer_id, SparkMax.MotorType.kBrushless)
        self.indexer.configure(
            self.indexer_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )

        self._state = ShooterStates.IDLE

        self.timer = Timer()

        self.pivot_voltage_request = VoltageOut(0.0)
        self.flywheel_voltage_request = VoltageOut(0.0)

        self.config = config

        self.io = ShooterIO()
        self._set_up_logging()

        self.pivot_motor.set_position(
            (self.angle_absolute_encoder.get() - ABS_ENCODER_OFFSET)
            / self.config.pivot_gear_ratio
        )
        self.pivot_pid.reset(self.get_pivot_angle())

        self.pid_sim = SimplePControllerSim(self.pivot_pid, None)
        self.mech_ligament = mech_root.appendLigament(
            "Shooter ligament", 1.5, self.pid_sim.value, color=Color8Bit(255, 0, 0)
        )

    @property
    def state(self) -> ShooterStates:
        return self._state

    @state.setter
    def state(self, state: ShooterStates) -> None:
        if not isinstance(state, ShooterStates):
            raise ValueError("State must be of type ShooterStates")
        self._state = state

    def go_to_idle(self) -> None:
        self.state = ShooterStates.IDLE

    def shoot(self) -> None:
        self.state = ShooterStates.AIMING

    def feed(self) -> None:
        self.state = ShooterStates.FEEDING

    def eject(self) -> None:
        self.state = ShooterStates.EJECT

    def has_note(self) -> bool:
        return not self.beam_break.get()

    def get_abs_position(self) -> float:
        return self.angle_absolute_encoder.get() - ABS_ENCODER_OFFSET

    def get_pivot_angle(self) -> units.radians:
        return (
            self.io.pivot_angle_supplier.value * self.config.pivot_gear_ratio * math.tau
        )

    def get_pivot_speed(self) -> units.radians_per_second:
        return (
            self.io.pivot_angular_velocity_supplier.value
            * self.config.pivot_gear_ratio
            * math.tau
        )

    def get_flywheel_speed(
        self,
    ) -> (
        units.radians_per_second
    ):  # TODO: check but I'm pretty sure this should be rad/s
        return (
            self.io.flywheel_velocity_supplier.value
            * self.config.shooter_gear_ratio
            * math.tau
        )

    def at_target_position(self) -> bool:
        if RobotBase.isSimulation():
            return self.pid_sim.at_goal()
        return self.pivot_pid.atGoal()

    def at_target_speed(self) -> bool:
        return self.get_flywheel_speed() >= self.io.target_flywheel_speed

    def execute(self):
        # TODO: Implement logic with io for sport mode
        self._handle_state_logic()

        self.io.target_shooter_angle = clamp(
            self.io.target_shooter_angle,
            MIN_SHOOTER_ANGLE,
            MAX_SHOOTER_ANGLE,
        )
        self.io.target_flywheel_speed = clamp(
            self.io.target_flywheel_speed, -MAX_SHOOTER_SPEED, MAX_SHOOTER_SPEED
        )
        self.io.indexer_voltage = clamp(self.io.indexer_voltage, -12, 12)

        if (
            self.get_pivot_angle() >= MIN_SHOOTER_ANGLE
            and self.get_pivot_angle() <= MAX_SHOOTER_ANGLE
        ):
            self.pivot_pid.setGoal(self.io.target_shooter_angle)
            self.io.pivot_voltage = self.pivot_pid.calculate(
                self.get_pivot_angle()
            ) + self.pivot_ff.calculate(self.get_pivot_angle(), 0.0)

            self.io.flywheel_voltage = self.flywheel_ff.calculate(
                self.io.target_flywheel_speed
            )

        elif self.get_pivot_angle() >= MAX_SHOOTER_ANGLE:
            self.io.pivot_voltage = 0.0
            self.io.flywheel_voltage = 0.0

        else:
            self.io.pivot_voltage = 2.0  # Moves the shooter slowly up until in bounds
            self.io.flywheel_voltage = 0.0

        if RobotBase.isSimulation():
            self.pid_sim.update()
            self.mech_ligament.setAngle(math.degrees(self.pid_sim.value))

        self.pivot_motor.set_control(
            self.pivot_voltage_request.with_output(
                self.io.pivot_voltage
            ).with_enable_foc(False)
        )
        self.top_flywheel_motor.set_control(
            self.flywheel_voltage_request.with_output(
                self.io.flywheel_voltage
            ).with_enable_foc(False)
        )
        self.bottom_flywheel_motor.set_control(
            self.flywheel_voltage_request.with_output(
                self.io.flywheel_voltage
            ).with_enable_foc(False)
        )
        self.indexer.setVoltage(self.io.indexer_voltage)

    def _handle_state_logic(self) -> None:
        if self.manual_tuning_mode and not self.io.tuning_sendables_sent:
            SmartDashboard.putData("Shooter Pivot PID", self.pivot_pid)
            self.io.tuning_sendables_sent = True

        if self.manual_tuning_mode:
            self.io.target_shooter_angle = self.pivot_pid.getGoal().position
            self.io.target_flywheel_speed = self.manual_flywheel_speed
            self.io.indexer_voltage = 0
            self.io.state = "Manual Tuning"
            return

        self.io.state = self.state.name

        match self.state:
            case ShooterStates.IDLE:
                self.io.target_shooter_angle = FEEDING_ANGLE
                self.io.indexer_voltage = 0
                self.io.target_flywheel_speed = 0

            case ShooterStates.FEEDING:
                self.io.target_shooter_angle = FEEDING_ANGLE
                self.io.indexer_voltage = INDEXER_FEED_VOLTAGE
                self.io.target_flywheel_speed = 0

                if self.has_note():
                    if not self.timer.isRunning():
                        self.timer.restart()
                    elif (
                        self.timer.get() >= FEED_DELAY
                    ):
                        self.state = ShooterStates.HOLDING
                        self.io.indexer_voltage = 0
                        self.timer.stop()

            case ShooterStates.HOLDING:
                self.io.target_shooter_angle = HOLDING_ANGLE
                self.io.indexer_voltage = 0
                self.io.target_flywheel_speed = 0

            case ShooterStates.AIMING:
                self.io.target_shooter_angle = SUBWOOFER_SHOOTING_ANGLE
                self.io.indexer_voltage = 0
                self.io.target_flywheel_speed = SHOOTER_SHOOTING_SPEED
                if self.at_target_position() and self.at_target_speed():
                    self.state = ShooterStates.SHOOTING

            case ShooterStates.SHOOTING:
                self.io.target_shooter_angle = SUBWOOFER_SHOOTING_ANGLE
                self.io.target_flywheel_speed = SHOOTER_SHOOTING_SPEED
                self.io.indexer_voltage = INDEXER_SHOOTING_VOLTAGE
            
                if not self.has_note():
                    self.state = ShooterStates.IDLE

            case ShooterStates.EJECT:
                self.io.target_shooter_angle = 0
                self.io.target_flywheel_speed = 50

                if self.get_flywheel_speed() > 30:
                    self.io.indexer_voltage = INDEXER_SHOOTING_VOLTAGE
                else:
                    self.io.indexer_voltage = 0

                if not self.timer.isRunning():
                    self.timer.restart()
                elif self.timer.get() >= 1.2:
                    self.state = ShooterStates.IDLE
                    self.timer.stop()

    def _set_up_logging(self) -> None:
        self.io.pivot_angle_supplier = self.pivot_motor.get_position()
        self.io.pivot_angular_velocity_supplier = self.pivot_motor.get_velocity()
        self.io.flywheel_velocity_supplier = self.top_flywheel_motor.get_velocity()

        self.io.add_function(self.has_note)
        self.io.add_function(self.get_pivot_angle, math.degrees)
        self.io.add_function(self.get_pivot_speed)
        self.io.add_function(self.at_target_position)
        self.io.add_function(self.get_flywheel_speed)
        self.io.add_function(self.at_target_speed)
        self.io.add_function(self.get_abs_position)

        self.pivot_pid.setGoal(self.io.target_shooter_angle)

        BaseStatusSignal.set_update_frequency_for_all(
            250,
            self.io.pivot_angle_supplier,
            self.io.pivot_angular_velocity_supplier,
            self.io.flywheel_velocity_supplier,
        )
