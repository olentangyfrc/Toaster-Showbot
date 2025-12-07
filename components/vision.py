import math
from collections.abc import Collection, Sequence
from logging import Logger
from typing import Any

from magicbot import tunable

# from ntcore.util import ChooserControl
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import (
    DriverStation,
    Field2d,
    RobotBase,
    SendableChooser,
    SmartDashboard,
    Timer,
)
from wpimath.geometry import Pose2d

from components.drivetrain import Drivetrain
from utilities.configs import VisionConfig
from utilities.elasticlib import Notification, NotificationManager
from utilities.helpers import get_struct_string
from utilities.IO import VisionIO
from utilities.vision_utils import (
    VisionMeasurement,
    get_recent_vision_measurements,
    get_vision_measurements_from_photoncamera,
    set_robot_orientation,
)

if RobotBase.isSimulation():
    from photonlibpy import PhotonCamera


class Vision:
    drivetrain: Drivetrain
    logger: Logger

    send_yaw_rate = tunable(True)

    def __init__(self, config: VisionConfig) -> None:
        """
        Initializes the vision subsystem by creating a networktables instance with all given limelights.
        """
        self.io = VisionIO()
        self.config = config
        self.camera_len = (
            len(self.config.camera_names)
            if isinstance(self.config.camera_names, Collection)
            else 1
        )

        if isinstance(self.config.camera_names, str):
            self.config.camera_names = [self.config.camera_names]

        self.measurement_tracker: dict[str, list[VisionMeasurement]] = {
            name: [] for name in config.camera_names
        }

        self.vision_field = Field2d()
        self.vision_field.setRobotPose(Pose2d())

        self.mt2_nt_topic = (
            "botpose_orb_wpiblue"
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue
            else "botpose_orb_wpired"
        )

        if RobotBase.isSimulation():
            self.apriltag_layout = AprilTagFieldLayout.loadField(
                AprilTagField.k2025ReefscapeAndyMark
            )

            self.sim_photon_camera_lookup = {
                name: PhotonCamera(name) for name in self.config.camera_names
            }

        self._set_up_logging()

    def _set_up_logging(self):
        self.disable_chooser = SendableChooser()
        self.disable_chooser.setDefaultOption("None", None)
        self.disable_chooser.addOption("all", "all")

        for name in self.measurement_tracker:
            self.disable_chooser.addOption(name, name)

        self.disable_chooser.onChange(lambda x: send(x))

        SmartDashboard.putData("Disable Chooser", self.disable_chooser)
        SmartDashboard.putData("Vision Field", self.vision_field)

        self.io.add_function(self.get_fused_pose)
        self.io.add_function(
            lambda: sum(self.get_recent_fps(name) for name in self.measurement_tracker)
            / self.camera_len,
            alternate_name="average FPS",
        )

        def send(x: Any):
            nonlocal self  # Fixes some scope issues

            if x not in [None, "all"]:
                NotificationManager.add_unconditional_notification(
                    Notification(
                        title="Limelight disabled",
                        description=f"{x} will not feed any more pose info",
                    )
                )
            elif x == "all":
                NotificationManager.add_unconditional_notification(
                    Notification(
                        title="All limelights disabled",
                        description="Pose info will solely be based off odometery now"
                    )
                )
    def execute(self) -> None:
        """
        Run every auton and teleop periodic cycle. Feeds vision measurements to the drivetrain to update odometry
        """
        for i, camera_name in enumerate(self.config.camera_names):
            if (
                camera_name == self.disable_chooser.getSelected()
                or self.disable_chooser.getSelected() == "all"
            ):
                continue

            tracker = self.measurement_tracker[camera_name]

            set_robot_orientation(
                camera_name,
                self.drivetrain.get_gyro_rotation().degrees(),
                self.drivetrain.get_gyro_velocity() * self.send_yaw_rate,
            )
            recent_measurements = (
                get_recent_vision_measurements(camera_name, self.mt2_nt_topic)
                if not RobotBase.isSimulation()
                else get_vision_measurements_from_photoncamera(
                    self.sim_photon_camera_lookup[camera_name],
                    self.drivetrain.get_gyro_rotation(),
                    self.config.robot_to_camera_transformations[i],
                    self.apriltag_layout,
                )
            )

            for measurement in recent_measurements:
                tracker.append(measurement)
                self.drivetrain.add_vision_measurement(measurement)
                self.vision_field.setRobotPose(measurement.pose)

            for i, measurement in enumerate(tracker):
                if (
                    Timer.getFPGATimestamp() - measurement.timestamp
                    < self.config.time_delay
                ):
                    tracker = tracker[i:]
                    break

            if (
                self.config.max_frame_count is not None
                and len(tracker) > self.config.max_frame_count
            ):
                index = len(tracker) - self.config.max_frame_count
                tracker = tracker[index:]

    def get_recent_pose(self, limelight_name: str) -> Pose2d:
        """
        Gets the most recent vision measurement for the given limelight name

        Args:
            limelight_name (str): Name of the limelight to measure from

        Returns:
            Pose2d: The most recent measured pose from the given limelight
        """
        if limelight_name not in self.measurement_tracker:
            self.logger.warning("%s is not a valid limelight name", limelight_name)
            return Pose2d()
        if not self.measurement_tracker[limelight_name]:
            return Pose2d()

        return self.measurement_tracker[limelight_name][-1].pose

    def get_recent_fps(self, limelight_name: str) -> float:
        """
        Gets the number of valid, processed vision measurements from the specific limelight in the last second

        Args:
            limelight_name (str): Name of the limelight to measure from

        Returns:
            int: The number of frames measured from in the last second
        """
        if limelight_name not in self.measurement_tracker:
            self.logger.warning("%s is not a valid limelight name", limelight_name)
            return 0
        return len(self.measurement_tracker[limelight_name]) / self.config.time_delay

    def get_fused_pose(self) -> Pose2d:
        """
        Gets the average pose reading for all cameras on the bot

        Returns:
            Pose2d: Pose2d object representing the average camera pose reading
        """
        self.io.most_recent_pose_measurements = tuple(
            self.get_recent_pose(name) for name in self.config.camera_names
        )
        fused_pose = Vision.get_average_pose(self.io.most_recent_pose_measurements)
        self.io.fused_pose_str = get_struct_string(fused_pose)

        return fused_pose

    # TODO: maybe move this into helpers
    @staticmethod
    def get_average_pose(poses: Sequence[Pose2d]):
        if not poses:
            return Pose2d()

        length = len(poses)

        if length == 1:
            return poses[0]

        x_sum = 0.0
        y_sum = 0.0

        sin_sum = 0.0
        cos_sum = 0.0

        for pose in poses:
            x_sum += pose.X()
            y_sum += pose.Y()

            sin_sum += pose.rotation().sin()
            cos_sum += pose.rotation().cos()

        avg_rot = math.atan2(sin_sum / length, cos_sum / length)
        avg_x = x_sum / length
        avg_y = y_sum / length

        return Pose2d(avg_x, avg_y, avg_rot)
