import math

import wpimath.units as units
from phoenix6 import CANBus
from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpilib import RobotBase
from wpimath.controller import (
    PIDController,
    ProfiledPIDController,
    SimpleMotorFeedforwardMeters,
)
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.trajectory import TrapezoidProfile

from components.modules.module import Module
from utilities.configs import SwerveConfig
from utilities.IO import SwerveIO


class GenericTalonFXModule(Module):
    def __init__(
        self,
        drive_motor_id: int,
        steer_motor_id: int,
        steer_encoder_id: int,
        offset: float,
        config: SwerveConfig,
        name: str,
        CANbus: CANBus,
        drive_inverted: bool,
        steer_inverted: bool,
    ) -> None:
        super().__init__(
            drive_motor_id,
            steer_motor_id,
            steer_encoder_id,
            offset,
            config,
            name,
            CANbus,
            drive_inverted,
            steer_inverted,
        )

        self.io = SwerveIO(name)
        self.offset = offset

        drive_configs = TalonFXConfiguration()
        drive_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        if RobotBase.isReal():
            drive_configs.current_limits.stator_current_limit = 50
            drive_configs.current_limits.stator_current_limit_enable = True

        if drive_inverted:
            drive_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            drive_configs.motor_output.inverted = (
                InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )

        self.drive_motor = TalonFX(drive_motor_id, CANbus)
        self.drive_motor.clear_sticky_faults()
        self.drive_motor.configurator.apply(drive_configs)  # type: ignore

        steer_configs = TalonFXConfiguration()
        steer_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        if steer_inverted:
            steer_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            steer_configs.motor_output.inverted = (
                InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )

        steer_configs.future_proof_configs = True

        self.steer_motor = TalonFX(steer_motor_id, CANbus)
        self.steer_motor.clear_sticky_faults()
        self.steer_motor.configurator.apply(steer_configs)  # type: ignore

        self.steer_encoder = CANcoder(steer_encoder_id, CANbus)
        self.steer_encoder.clear_sticky_faults()

        self.steer_motor.stopMotor()
        self.drive_motor.stopMotor()

        self.feed_forward = SimpleMotorFeedforwardMeters(
            self.config.ff_constants.kS,
            self.config.ff_constants.kV,
            self.config.ff_constants.kA,
        )

        self.drive_pid = PIDController(
            self.config.drive_pid_constants.p,
            self.config.drive_pid_constants.i,
            self.config.drive_pid_constants.d,
        )

        self.steer_pid = ProfiledPIDController(
            self.config.steer_pid_constants.p,
            self.config.steer_pid_constants.i,
            self.config.steer_pid_constants.d,
            TrapezoidProfile.Constraints(9999, 9999),
        )

        self.steer_pid.enableContinuousInput(-math.pi, math.pi)
        self.steer_pid.setTolerance(math.radians(1))

        self.drive_request = VoltageOut(0)
        self.steer_request = VoltageOut(0)

        self.io.position_supplier = self.drive_motor.get_position()
        self.io.velocity_supplier = self.drive_motor.get_velocity()
        self.io.angle_supplier = self.steer_encoder.get_absolute_position()
        self.io.angular_velocity_supplier = self.steer_encoder.get_velocity()

        BaseStatusSignal.set_update_frequency_for_all(
            250,
            self.io.position_supplier,
            self.io.velocity_supplier,
            self.io.angle_supplier,
            self.io.angular_velocity_supplier,
        )

        self._set_up_logging()

    def _set_up_logging(self) -> None:
        self.io.offset = self.offset
        self.io.add_function(self.get_module_voltage)
        self.io.add_function(self.get_magnitude)

        # self.io.add_ctre_device(
        #     self.drive_motor,
        #     self.steer_motor,
        #     self.steer_encoder,
        #     names=(
        #         self.name + " drive motor",
        #         self.name + " steer motor",
        #         self.name + " encoder",
        #     ),
        # )

    def get_drive_position(self) -> units.meters:
        return (
            BaseStatusSignal.get_latency_compensated_value(
                self.io.position_supplier, self.io.velocity_supplier
            )
            * self.config.drive_ratio
            * self.config.wheel_radius
            * math.tau
        )

    def get_drive_velocity(self) -> units.meters_per_second:
        # For some reason latency compensation causes issues in sim. I'll leave
        # the code here for irl testing though

        # vel = BaseStatusSignal.get_latency_compensated_value(
        #     self.drive_motor.get_velocity(),
        #     self.drive_motor.get_acceleration()
        # ) * self.config.drive_ratio * self.config.wheel_radius * math.tau
        return (
            self.io.velocity_supplier.value
            * self.config.drive_ratio
            * self.config.wheel_radius
            * math.tau
        )

    def get_magnitude(self) -> units.meters_per_second:
        self.io.module_speed = math.hypot(
            self.drive_motor.get_velocity().value, self.steer_motor.get_velocity().value
        )
        return self.io.module_speed

    def get_angle(self) -> units.radians:
        return (
            self.io.angle_supplier.value * math.tau
            - math.radians(self.offset) * RobotBase.isReal()
        )

    def get_module_voltage(self) -> tuple[units.volts, units.volts]:
        return (
            self.drive_motor.get_motor_voltage().value,
            self.steer_motor.get_motor_voltage().value,
        )

    def stop(self) -> None:
        self.drive_motor.stopMotor()
        self.steer_motor.stopMotor()

    def reset(self) -> None:
        self.drive_motor.set_position(0)
        self.steer_motor.set_position(0)

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        rot = Rotation2d(self.get_angle())

        desired_state.optimize(rot)
        desired_state.cosineScale(rot)

        self.io.desired_speed = desired_state.speed
        self.io.desired_angle = desired_state.angle.degrees()

        drive_volts = self.feed_forward.calculate(
            desired_state.speed
        ) + self.drive_pid.calculate(self.get_drive_velocity(), desired_state.speed)

        steer_volts = self.steer_pid.calculate(
            rot.radians(), desired_state.angle.radians()
        )

        self.drive_motor.set_control(
            self.drive_request.with_output(drive_volts).with_enable_foc(True)
        )
        self.steer_motor.set_control(
            self.steer_request.with_output(steer_volts).with_enable_foc(True)
        )

    def manually_control(self) -> None:
        self.set_desired_state(
            SwerveModuleState(
                0, Rotation2d(self.steer_pid.getGoal().position % math.tau)
            )
        )
