from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import wpimath.units as units
from phoenix6 import CANBus

from utilities.helpers import FREE_SPEED_LOOKUP, FFConstants, MotorTypes, PIDConstants

# This deals with circular imports for type annotations
if TYPE_CHECKING:
    from components.modules.module import (
        Module,
    )


@dataclass
class SwerveConfig:
    drive_ratio: float
    steer_ratio: float
    steer_pid_constants: PIDConstants
    drive_pid_constants: PIDConstants
    ff_constants: FFConstants
    wheel_radius: units.meters
    drive_motor_type: MotorTypes

    @property
    def max_module_speed(self) -> float:
        return (
            FREE_SPEED_LOOKUP.get(self.drive_motor_type, 100)
            * self.drive_ratio
            * self.wheel_radius
            * math.tau
        )


@dataclass
class DrivetrainConfig:
    drive_motor_ids: tuple[int, int, int, int]
    steer_motor_ids: tuple[int, int, int, int]
    steer_encoder_ids: tuple[int, int, int, int]
    gyro_id: int
    module_offsets: tuple[float, float, float, float]
    drive_inverted: tuple[bool, bool, bool, bool]
    steer_inverted: tuple[bool, bool, bool, bool]
    max_translation_speed: units.meters_per_second
    max_rotation_speed: units.meters_per_second
    enable_discretization: bool
    enable_passive_snap: bool
    automatically_lock: bool
    wheel_base: units.meters
    track_width: units.meters
    rotation_pid: PIDConstants
    translation_pid: PIDConstants
    auto_pid: PIDConstants
    module_type: type[Module]
    swerve_config: SwerveConfig
    CANbus: CANBus


@dataclass
class IntakeConfig:
    roller_id: int
    beam_break_id: int
    pivot_motor_id: int
    gear_ratio: float
    pivot_ff: FFConstants
    pivot_pid: PIDConstants
    pivot_max_vel: units.radians_per_second
    pivot_max_acc: units.radians_per_second_squared
    CANbus: CANBus
