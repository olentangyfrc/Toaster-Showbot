# type: ignore
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from itertools import zip_longest
from typing import TYPE_CHECKING, Callable

import phoenix6.unmanaged
import wpimath.units as units
from phoenix6.hardware import TalonFX
from phoenix6.sim import ChassisReference
from pyfrc.physics.core import PhysicsInterface
from wpilib import RobotBase, RobotController
from wpilib.simulation import (
    BatterySim,
    DCMotorSim,
    ElevatorSim,
    RoboRioSim,
    SingleJointedArmSim,
)
from wpimath.system.plant import DCMotor, LinearSystemId

from components.modules.generic_talon_fx_module import GenericTalonFXModule
from utilities import helpers as utils

if TYPE_CHECKING:
    from robot import MyRobot

START_VOLTS = 12.6


class SimSystem(ABC):
    @abstractmethod
    def update(self, dt: units.seconds) -> None: ...


class TalonFXMotorSim(SimSystem):
    def __init__(
        self,
        motor_callable: Callable[[int], DCMotor],
        *motors: TalonFX,
        moi: units.kilogram_square_meters,
        gearing: float,
        inverted: list[bool] | None = None,
    ) -> None:
        gearbox = motor_callable(len(motors))
        self.sim_motor = DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, moi, gearing), gearbox
        )
        self.gearing = gearing

        self.sim_states = [motor.sim_state for motor in motors]

        for sim_state, invert in zip_longest(
            self.sim_states, inverted or [], fillvalue=False
        ):
            sim_state.set_supply_voltage(12)
            if invert:
                sim_state.orientation = ChassisReference.Clockwise_Positive
            else:
                sim_state.orientation = ChassisReference.CounterClockwise_Positive

    def update(self, dt: units.seconds) -> None:
        ratio = self.gearing / math.tau
        volts = self.sim_states[0].motor_voltage

        self.sim_motor.update(dt)
        self.sim_motor.setInputVoltage(volts)

        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.sim_motor.getAngularPosition() * ratio
            )
            sim_state.set_rotor_velocity(self.sim_motor.getAngularVelocity() * ratio)
            sim_state.set_rotor_acceleration(
                self.sim_motor.getAngularAcceleration() * ratio
            )


class TalonFXArmSim(SimSystem):
    def __init__(
        self,
        mech: SingleJointedArmSim,
        *motors: TalonFX,
        gearing: float,
        inverted: list[bool] | None = None,
    ) -> None:
        self.mech = mech
        self.gearing = gearing

        self.sim_states = [motor.sim_state for motor in motors]

        for sim_state, invert in zip_longest(
            self.sim_states, inverted or [], fillvalue=False
        ):
            sim_state.set_supply_voltage(12)
            if invert:
                sim_state.orientation = ChassisReference.Clockwise_Positive
            else:
                sim_state.orientation = ChassisReference.CounterClockwise_Positive

    def update(self, dt: units.seconds) -> None:
        ratio = self.gearing / math.tau
        volts = self.sim_states[0].motor_voltage

        self.mech.setInputVoltage(volts)
        self.mech.update(dt)

        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(self.mech.getAngle() * ratio)
            sim_state.set_rotor_velocity(self.mech.getVelocity() * ratio)


class TalonFXElevatorSim(SimSystem):
    def __init__(
        self,
        mech: ElevatorSim,
        *motors: TalonFX,
        gearing: float,
        inverted: list[bool] | None = None,
    ) -> None:
        self.elevator_sim = mech
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]

        for sim_state, invert in zip_longest(
            self.sim_states, inverted or [], fillvalue=False
        ):
            sim_state.set_supply_voltage(12)
            if invert:
                sim_state.orientation = ChassisReference.Clockwise_Positive
            else:
                sim_state.orientation = ChassisReference.CounterClockwise_Positive

    def update(self, dt: units.seconds) -> None:
        volts = self.sim_states[0].motor_voltage

        self.elevator_sim.setInputVoltage(volts)
        self.elevator_sim.update(dt)

        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.elevator_sim.getPosition() * self.gearing
            )
            sim_state.set_rotor_velocity(self.elevator_sim.getVelocity() * self.gearing)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot) -> None:
        self.physics_controller = physics_controller

        self.drivetrain = robot.drivetrain

        self.modules: tuple[
            GenericTalonFXModule,
            GenericTalonFXModule,
            GenericTalonFXModule,
            GenericTalonFXModule,
        ] = self.drivetrain.modules

        self.drives = [
            TalonFXMotorSim(
                DCMotor.krakenX60FOC,
                module.drive_motor,
                moi=0.001,
                gearing=1 / robot.drivetrain_config.swerve_config.drive_ratio,
                inverted=[module.drive_inverted],
            )
            for module in self.modules
        ]

        self.steers = [
            TalonFXMotorSim(
                DCMotor.krakenX60,
                module.steer_motor,
                moi=0.001,
                gearing=1 / robot.drivetrain_config.swerve_config.steer_ratio,
                inverted=[module.steer_inverted],
            )
            for module in self.modules
        ]

        self.encoders = [module.steer_encoder.sim_state for module in self.modules]
        for encoder in self.encoders:
            encoder.set_supply_voltage(RobotController.getBatteryVoltage())

        self.gyro = robot.drivetrain.gyro.sim_state

        self.current_draws = []

    def update_sim(self, now: float, tm_diff: float) -> None:
        if RobotBase.isSimulation():
            phoenix6.unmanaged.feed_enable(0.1)

        for drive, steer, encoder in zip(self.drives, self.steers, self.encoders):
            drive.update(tm_diff)
            steer.update(tm_diff)

            encoder.set_raw_position(steer.sim_motor.getAngularPosition() / math.tau)
            encoder.set_velocity(steer.sim_motor.getAngularVelocity() / math.tau)

            # Need to clamp otherwise current goes insane (clamped to stator current limits)
            self.current_draws.extend(
                [
                    utils.clamp(abs(sim_state.supply_current), 0, 50)
                    for sim_state in drive.sim_states
                ]
            )
            self.current_draws.extend(
                [
                    utils.clamp(abs(sim_state.supply_current), 0, 50)
                    for sim_state in steer.sim_states
                ]
            )

        speeds = self.drivetrain.kinematics.toChassisSpeeds(
            self.drivetrain.get_module_states()
        )

        self.gyro.add_yaw(math.degrees(speeds.omega * tm_diff))
        self.physics_controller.drive(speeds, tm_diff)

        self.apply_voltage()
        self.current_draws.clear()

    def apply_voltage(self) -> None:
        RoboRioSim.setVInVoltage(BatterySim.calculate(self.current_draws))
        new_volts = RobotController.getBatteryVoltage()

        for drive, steer, encoder in zip(self.drives, self.steers, self.encoders):
            encoder.set_supply_voltage(new_volts)
            [sim_state.set_supply_voltage(new_volts) for sim_state in drive.sim_states]
            [sim_state.set_supply_voltage(new_volts) for sim_state in steer.sim_states]

        self.gyro.set_supply_voltage(new_volts)
