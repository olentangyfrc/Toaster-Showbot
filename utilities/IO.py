from choreo.trajectory import SwerveSample, SwerveTrajectory
from phoenix6.status_signal import StatusSignal
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState

from utilities.IO_helpers import IO, PublisherSpecifications, auto_log

RATIO = 0.09588832710352932  # Gear Ratio for SwerveModule


@auto_log
class SwerveIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        groups={"*supplier": "motor data"},
        container="Modules",
        ignore_fields={
            "angular_velocity_supplier",
        },
        modifiers={
            "angle_supplier": lambda _: _ * 360 % 360,
            "velocity_supplier": lambda _: _ * RATIO,
        },
    )

    position_supplier: StatusSignal[float]
    velocity_supplier: StatusSignal[float]
    angle_supplier: StatusSignal[float]
    angular_velocity_supplier: StatusSignal[float]

    def __init__(self, name: str) -> None:
        super().__init__(name)


@auto_log
class DrivetrainIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        groups={
            ("sample*", "auto*"): "auto",
            ("target*", "odometery*", "*future_pose*"): "pose info/other",
            "stable*": "pose info/rotation readings",
            "pose*": "pose info",
            ("applied*", "actual*", "commanded*"): "speeds",
            ("*module*"): "module data",
        },
        ignore_fields={
            "*signal",
            "last*",
            "gyro_yaw_supplier",
            "gyro_angular_speed_supplier",
            "stable*",
            "*actual",
        },
        conditions={
            ("auto*", "sample*"): DriverStation.isAutonomous,
            (
                "commanded_speed",
                "field_relative",
                "tuning*",
            ): DriverStation.isTeleop,  # applied should be same as sample
        },
        high_resolution={"pose*", "sample*", "*module_states"},
        modifiers={"target_rotation": lambda _: _.degrees() % 360},
    )

    gyro_yaw_supplier: StatusSignal[float]
    gyro_angular_speed_supplier: StatusSignal[float]

    def __init__(self) -> None:
        self.state = "None"

        self.applied_speed = ChassisSpeeds()
        self.field_relative = True

        # The speed that the robot is actually driving at
        self.actual_speed = ChassisSpeeds()

        self.pose_str = ""
        self.odometery_pose = Pose2d()

        self.target_pose = Pose2d()
        self.target_rotation = Rotation2d()

        self.desired_module_states = (
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
        )

        self.auto_px = 0.0
        self.auto_py = 0.0
        self.auto_pomega = 0.0

        self._sample_pose = Pose2d()

        self._sample_speed = ChassisSpeeds()

        self._sample_ax = 0.0
        self._sample_ay = 0.0
        self._sample_alpha = 0.0

        self._sample_fx = 0.0
        self._sample_vy = 0.0

        # Stuff I kind of stuffed here to keep drivetrain.py cleaner

        self.stable_rotation = (
            Rotation2d()
        )  # Rotation reading that persists even if gyro goes down
        self.stable_rotation_deg = 0.0
        self.stable_angular_velocity = 0.0

        self.last_module_positions = (
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
        )

        self.last_auton_trajectory = SwerveTrajectory("", [], [], [])
        self.tuning_senables_sent = False

        super().__init__()

    @property
    def sample(self) -> AttributeError:
        """Write only property. Sets a SwerveSample object to log multiple values from with a simpler syntax"""
        raise AttributeError("Sample property is write only. Do not access it.")

    @sample.setter
    def sample(self, sample: SwerveSample) -> None:
        self._sample_pose = sample.get_pose()

        self._sample_speed = sample.get_chassis_speeds()

        self._sample_ax = sample.ax
        self._sample_ay = sample.ay

        self._sample_fx = sample.fx
        self._sample_fy = sample.fy

        self._sample_omega = sample.omega
        self._sample_alpha = sample.alpha


@auto_log
class RobotIO(IO):
    def __init__(self) -> None:
        self.voltage = 0.0
        self.network_usage = 0.0
        self.cpu_temp = 0.0
        self.is_browned_out = False
        self.time = 0.0
        super().__init__()


@auto_log
class IntakeIO(IO):
    SPECIFICATIONS = PublisherSpecifications(
        ignore_fields={
            "pivot_position_supplier",
            "pivot_velocity_supplier",
            "tuning_sendables_sent",
        }
    )

    pivot_position_supplier: StatusSignal[float]
    pivot_velocity_supplier: StatusSignal[float]

    def __init__(self) -> None:
        self.state = "IDLE"
        self.target_pivot_position = 0.0
        self.target_roller_voltage = 0.0

        self.tuning_sendables_sent = False

        super().__init__()
