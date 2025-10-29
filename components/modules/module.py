from abc import ABC, abstractmethod

import wpimath.units as units
from phoenix6 import CANBus
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from utilities.configs import SwerveConfig


class Module(ABC):
    """Abstract base class for all swerve modules.

    Don't instantiate this directly. Instead this should be inherited from and be a
    set of directions when creating other swerve module classes to mantain functionality
    between different modules.
    """

    def __init__(
        self,
        drive_motor_id: int,
        steer_motor_id: int,
        steer_encoder_id: int,
        offset: float,
        config: SwerveConfig,
        name: str,
        canbus: CANBus,
        drive_inverted: bool,
        steer_inverted: bool,
    ) -> None:
        # All these need to be attributes for other parts of the code to work
        # so ensuring they all are over here.
        self.drive_inverted = drive_inverted
        self.steer_inverted = steer_inverted
        self.config = config
        self.name = name
        super().__init__()

    @abstractmethod
    def get_drive_position(self) -> units.meters:
        """Returns the position of the drive motor.

        This should return the relative, not absolute, position of the drive motor. So
        rotations in different directions should count as positive or negative. This method
        is essential to calculating the swerve module position which can be used in higher level
        odometery tracking.

        Returns:
            float (m): the relative position of the motor
        """

    @abstractmethod
    def get_drive_velocity(self) -> units.meters_per_second:
        """Returns the velocity of the drive motor.

        Since this is a measure of velocity, not speed, it must have a sign of direction. This method
        is essential to calculating the swerve module state which can be used in higher level
        odometery tracking.

        Returns:
            float (m/s): the velocity of the drive motor
        """

    @abstractmethod
    def get_magnitude(self) -> units.meters_per_second:
        """Returns the calculated magnitude of the swerve module's motion.

        The module's motion can be thought of as a combination of:
        - The linear movement from the drive motor.
        - The rotational movement from the steer motor.

        The magnitude is typically calculated as the vector magnitude of these two components:
            `math.hypot(drive_motor_velocity, steer_motor_velocity)`

        This can be useful for determining if the module (and thus the drivetrain)
        is completely stationary. Even if one motor is stopped, motion in the other motor
        will contribute to a non-zero magnitude.

        Returns:
            float (m/s): the combined magnitude of motion from the entire module.
        """

    @abstractmethod
    def get_angle(self) -> units.radians:
        """Returns the angle of the module.

        This can either be the angle from an external encoder, which does introduce some extra noise,
        or the integrated encoder on the steer motor if it has one. Note that integrated encoders will need
        to be zeroed using an absolute encoder to ensure consistency in position.

        This method is needed to calculate both the swerve module's state and position.

        Returns:
            float (radians): the current angle of the swerve module
        """

    @abstractmethod
    def get_module_voltage(self) -> tuple[units.volts, units.volts]:
        """Returns the voltages being applied to the drive and steer motor.

        This method is mainly for logging purposes. Since certain onboard controls (eg. MotionMagic) don't give you
        direct control over how much voltage is applied to each motor.

        Returns:
            tuple: the voltages applied to the drive and then steer motor.
        """

    @abstractmethod
    def manually_control(self) -> None:
        """Gives manual rotation control to the operator for tuning.

        This should set the state of the swerve module to have a speed of 0mps and have a rotation of whatever
        is inputted to the PID controller via dashboard. This method is only needed for modules that use
        wpilib PID controllers since onboard control can be tuned via PhoenixTuner or Rev Hardware Client.
        """
        pass  # Defaults to nothing so only classes that need it can implement it

    @abstractmethod
    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        """Sets the desired state of the swerve module.

        A swerve module state has two properties: angle and speed. This method
        uses both those attributes to command the swerve module to go to a certain
        angle and drive at a certain speed.

        Args:
            desired_state: the desired state for the swerve module, contains the commanded
                speed and rotation to drive at
        """

    @abstractmethod
    def reset(self) -> None:
        """Resets the current module.

        This should reset the distance traveled for both the drive and steer motors to zero. This is
        useful for resetting the drivetrain's position during the start of a match.
        """

    @abstractmethod
    def stop(self) -> None:
        """Stops the module.

        This should immediately set the speed of the drive and steer motors to zero. This is implemented to
        ensure safety, so someone can quickly stop the drivetrain.
        """

    def get_state(self) -> SwerveModuleState:
        """Returns the current state of the swerve module.

        Returns:
            SwerveModuleState: an object containing the velocity (mps) and angle (Rotation2d) of the module
        """
        return SwerveModuleState(
            self.get_drive_velocity(), Rotation2d(self.get_angle())
        )

    def get_position(self) -> SwerveModulePosition:
        """Returns the current position of the swerve module.

        Returns:
            SwerveModulePosition: an object containing the position (m) and angle (Rotation2d) of the module
        """
        return SwerveModulePosition(
            self.get_drive_position(), Rotation2d(self.get_angle())
        )
