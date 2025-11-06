import math
import re
from collections.abc import Callable
from enum import Enum
from typing import Any, ClassVar, NamedTuple, Optional, Protocol, runtime_checkable

import wpimath
import wpimath.units as units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds


class PIDConstants(NamedTuple):
    p: float
    i: float
    d: float
    max_speed: Optional[float] = None
    max_accel: Optional[float] = None


class FFConstants(NamedTuple):
    kS: float
    kV: float
    kA: float
    kG: Optional[float] = None


class ProfileConstants(NamedTuple):
    max_vel: float
    max_acc: float


class MotorTypes(Enum):
    KRAKEN_X60 = "Kraken X60"
    KRAKEN_X60_FOC = "Kraken X60 FOC"
    KRAKEN_X44 = "Kraken X44"
    KRAKEN_X44_FOC = "Kraken X44 FOC"
    FALCON_500 = "Falcon 500"
    FALCON_500_FOC = "Falcon 500 FOC"
    MINION = "Minion"
    MINION_FOC = "Minion FOC"
    NEO_550 = "NEO 550"
    NEO_VORTEX = "NEO Vortex"
    NEO_V1_1 = "NEO V1.1"


class DrivetrainStates(Enum):
    ACTIVE_SNAP = "Active Snap"
    PASSIVE_SNAP = "Passive Snap"
    LINE_UP = "Line up"
    LOCK = "Lock"


class DriveSignal:
    def __init__(
        self, speed: ChassisSpeeds | None = None, field_relative: bool = True
    ) -> None:
        if speed is None:
            speed = ChassisSpeeds()

        self._speed = speed
        self._field_relative = field_relative

    @property
    def speed(self) -> ChassisSpeeds:
        return self._speed

    @speed.setter
    def speed(self, speed: ChassisSpeeds) -> None:
        if not isinstance(speed, ChassisSpeeds):
            raise ValueError(
                "Drive signal speed can only be set to chassis speed object"
            )
        self._speed = speed

    def get_field_relative(self) -> bool:
        return self._field_relative

    def get_magnitude(self) -> units.meters_per_second:
        return math.hypot(self._speed.vx, self._speed.vy)


@runtime_checkable
class Struct(Protocol):
    WPIStruct: ClassVar[Any]

    def __repr__(self) -> str:  # All objects should have this, but just to make sure
        ...


FREE_SPEED_LOOKUP = {
    MotorTypes.KRAKEN_X60: 6000 / 60,
    MotorTypes.KRAKEN_X60_FOC: 5800 / 60,
    MotorTypes.KRAKEN_X44: 7530 / 60,
    # There's probably a different speed, but I couldn't find them anywhere
    MotorTypes.KRAKEN_X44_FOC: 7530 / 60,
    MotorTypes.FALCON_500: 6380 / 60,
    MotorTypes.FALCON_500_FOC: 6079 / 60,
    MotorTypes.MINION: 7200 / 60,
    MotorTypes.MINION_FOC: 7200 / 60,
    MotorTypes.NEO_VORTEX: 6784 / 60,
    MotorTypes.NEO_550: 11000 / 60,
    MotorTypes.NEO_V1_1: 5676 / 60,
}


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Restricts a value to a upper and lower bound.

    Values below the minimum will be set to the minimum value and values above
    the maximum will be set to the maximum. Otherwise, values will remain unchanged.
    This is typically used to make sure a variable cannot exceed the physical constraints of
    a mechanism.

    Args:
        value: the value to clamp.
        min_val: the lower bound to clamp to, values below this are clamped to it.
        max_val: the upper bound to clamp to, values above are clamped to it.

    Returns:
        float: the new value, ensured to be in the range [min_val, max_val].

    Raises:
        ValueError: If max_val is not greater than min_val.
    """
    if max_val <= min_val:
        raise ValueError("Max clamp value must be greater than min clamp value")
    return min(max_val, max(min_val, value))


def filter_input(controller_input: float, deadband: Optional[float] = 0.0225) -> float:
    """Filters joystick input from a Xbox or PS4 controller

    This squares input from the joystick, maintaining direction using a sign function, which makes
    the joystick need to be throttled further to reach a maximum value since joystick input is
    within the range [-1, 1]. If the optional deadband parameter is provided, any values below it
    will be set to zero. This helps counteract natural noise in the joysticks.

    Args:
        controller_input: the unfiltered input directly from a Xbox or PS4 controller joystick accessed using
            the wpilib classes.
        deadband (Optional): the minimum value to register inputs from. Decreasing this value will provide
            finer control but may increase the risk of noise from the joysticks interfering with driver control.
            If set to None, then no deadband will be applied and filtered input will only be the provided input
            squared.

    Returns:
        float: the filtered joystick input, calculated as:
            sign(controller_input) * controller_input^2 if greater than deadband else 0
    """
    controller_input_corrected = math.copysign(controller_input**2, controller_input)

    if deadband is not None:
        return wpimath.applyDeadband(controller_input_corrected, deadband)
    else:
        return controller_input_corrected


def inputModulus(input: float, min: float, max: float) -> float:
    """Wraps a value around a minimum and maximum value

    Creates a circular wrap around the minimum and maximum value. Any values greater than the
    maximum will circle around and increase from the minimum and any values less than the minimum
    will circle around and decrease from the maximum. This is helpful when dealing with angular motion
    since angles are inherently circular (370 degrees is the same as 10 degrees and -10 degrees is the same
    as 350 degrees).

    Example uses:
        ```
        inputModulus(150, 0, 100) -> 50
        inputModulus(-400, 0, 360) -> 320
        inputModulus(math.pi, 0, math.tau) -> math.pi
        ```

    Args:
        input: the value to be wrapped
        min: the minimum bound of the wrap. Values greater than max will be wrapped to this.
        max: the maximum bound of the wrap. Values less than min will be wrapped to this.

    Returns:
        float: the input value wrapped within the range [min, max]
    """
    modulus = max - min
    return ((input - min) % modulus) + min


def within_pose_tolerance(
    p1: Pose2d, p2: Pose2d, translation_tol: units.meters, rotation_tol: units.degrees
) -> bool:
    """Checks if two Pose2d objects are within tolerance with each other

    This checks if the X and Y of the poses are within one tolerance and if rotation is within another
    tolerance. This lets you provide laxer rotation tolerances to account for some of the noise of the
    gyro while still being strict on the translation.

    Args:
        p1: the first Pose2d to check
        p2: the second Pose2d to check
        translation_tol: the tolerance for the translational components of each pose to be within
        rotation_tol: the tolerance for the rotational components of each pose to be within

    Returns:
        bool: true if the poses are within tolerance of eachother else false
    """
    t = within_translation_tolerance(
        p1.translation(), p2.translation(), translation_tol
    )
    r = within_rotation_tolerance(p1.rotation(), p2.rotation(), rotation_tol)
    return t and r


def within_translation_tolerance(
    t1: Translation2d, t2: Translation2d, tolerance: units.meters
) -> bool:
    """Checks if two Translation2d objects are within tolerance of eachother

    This compares both the x and y components of the translations to make sure they fall within tolerance
    instead of comparing the euclidean distance between the two. This allows for more even tolerance checking,
    but tolerance must be lower to facilitate the fact that max distance error can be tolerance^2 technically.

    Args:
        t1: the first Translation2d to check
        t2: the second Translation2d to check
        tolerance: the tolerance for X and Y to be within

    Returns:
        bool: true if the translations are within tolerance false otherwise
    """
    return abs(t1.X() - t2.X()) <= tolerance and abs(t1.Y() - t2.Y()) <= tolerance


def within_rotation_tolerance(
    r1: Rotation2d, r2: Rotation2d, tolerance: units.degrees
) -> bool:
    """Checks if two Rotation2d objects are within tolerance of eachother

    This compares the two rotations directly. So a rotation of 720 degrees will not
    be considered within tolerance of a rotation of 360 degrees even if the two
    represent the same point on a circle.

    Args:
        r1: the first Rotation2d to check
        r2: the second Rotation2d to check
        tolerance: the tolerance for the rotations to be within

    Returns:
        bool: true if the rotations are within tolerance else false
    """
    return abs((r1 - r2).radians()) <= math.radians(tolerance)


def get_struct_string(struct: Struct) -> str:
    """Gets a filtered-string from a WPIStruct object.

    This is particularily useful for display on dashboards as it will get all the info needed
    in one topic instead of two or three. So it will take up less space on the actual dashboard
    itself, in memory, and less time to iterate though.

    **Only use this data on structs you want to display on the dashboard. It doesn't provide any use in replay**

    Args:
        struct (Struct): The struct (eg. a Rotation2d, Translation2d, Pose2d) to recieve the string from

    Raises:
        ValueError: the object passed in is not a Struct

    Returns:
        str: the data of the struct formatted as x: _, y: _, rot: _ if the object has any of those fields.
    """
    if not isinstance(struct, Struct):
        raise ValueError(f"{type(struct)} is not a WPIStruct")

    matches = re.findall(
        r"(x|y|z|dx|dy|dz|omega|distance)=([-+]?\d*\.\d+)|Rotation2d\(([-+]?\d*\.\d+)\)",
        repr(struct),
    )

    result = ""
    for match in matches:
        if match[0]:  # first check
            result += match[0] + ": " + _round_numeric_string(match[1]) + ", "
        elif match[2]:  # anything else
            result += "rot: " + _round_numeric_string(
                match[2], conversion=lambda x: math.degrees(x) % 360
            )
    return result


def _round_numeric_string(
    numeric_string: str,
    digits: int = 2,
    conversion: Callable[[float], float] | None = None,
) -> str:
    """Rounds a number-only string to a certain precision

    Args:
        numeric_string (str): The string to round. Must only have a decimal point and numbers
        digits (int, optional): The precision to round to. Defaults to 2.
        conversion (Callable, optional): A conversion factor to apply. Useful for unit conversion

    Returns:
        str: The string, with the rounded precision.
    """
    try:
        num = (
            conversion(float(numeric_string))
            if conversion is not None
            else float(numeric_string)
        )
        return str(round(num, digits))
    except Exception:
        return ""  # A error here won't crash everything
