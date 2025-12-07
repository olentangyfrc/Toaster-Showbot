from magicbot import MagicRobot
from ntcore import NetworkTableInstance
from phoenix6 import CANBus
from wpilib import (
    Color8Bit,
    DataLogManager,
    DriverStation,
    Mechanism2d,
    RobotController,
    SmartDashboard,
    Timer,
    XboxController,
)
from wpilib.deployinfo import getDeployData
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from components.climber import Climber
from components.drivetrain import DriveSignal, Drivetrain
from components.intake import Intake, IntakeStates
from components.modules.generic_talon_fx_module import GenericTalonFXModule
from components.shooter import Shooter, ShooterStates
from utilities import helpers as utils
from utilities.configs import (
    DrivetrainConfig,
    IntakeConfig,
    ShooterConfig,
    SwerveConfig,
)
from utilities.elasticlib import Notification, NotificationLevel, NotificationManager
from utilities.helpers import FFConstants, MotorTypes, PIDConstants, ProfileConstants
from utilities.IO_helpers import IO


class MyRobot(MagicRobot):
    drivetrain: Drivetrain
    climber: Climber
    intake: Intake
    shooter: Shooter

    def createObjects(self) -> None:
        DataLogManager.start()
        DataLogManager.logNetworkTables(True)
        DataLogManager.logConsoleOutput(True)
        DriverStation.startDataLog(DataLogManager.getLog())

        # Logs some meta data for advantagescope
        meta_table = NetworkTableInstance.getDefault().getTable("Metadata")
        deploy_info = getDeployData()

        if deploy_info is not None:
            for key, value in deploy_info.items():
                meta_table.putString(key, value)

        meta_table.putString("Runtime Type", self.getRuntimeType().name[1:])
        meta_table.putString("Serial Number", RobotController.getSerialNumber())

        self.controller = XboxController(0)
        self.aux_controller = XboxController(1)
        self.CANbus = CANBus("*")

        swerve_config = SwerveConfig(
            drive_ratio=1 / 7.7142857,
            steer_ratio=1 / 7.7142857 if self.isReal() else 1/25.9,
            steer_pid_constants=PIDConstants(3.7, 0, 0.05),
            drive_pid_constants=PIDConstants(0.5, 0, 0),
            ff_constants=FFConstants(0.2278, 2.4176, 0),
            wheel_radius=0.08592 / 2,
            drive_motor_type=MotorTypes.KRAKEN_X60_FOC,
        )

        self.drivetrain_config = DrivetrainConfig(
            drive_motor_ids=(20, 22, 24, 26),
            steer_motor_ids=(21, 23, 25, 27),
            steer_encoder_ids=(19, 18, 17, 16),
            gyro_id=51,
            module_offsets=(46.85, 0.5, 261.21, 57.04),
            drive_inverted=(False, False, False, False),  # True = clockwise positive
            steer_inverted=(True, True, True, True),
            max_translation_speed=4.59,
            max_rotation_speed=7,
            enable_discretization=False,
            enable_passive_snap=False,
            automatically_lock=False,
            wheel_base=0.584,
            track_width=0.584,
            rotation_pid=PIDConstants(0.0, 0, 0),
            translation_pid=PIDConstants(0.0, 0, 0),
            auto_pid=PIDConstants(0.0, 0, 0),
            module_type=GenericTalonFXModule,
            swerve_config=swerve_config,
            CANbus=self.CANbus,
        )

        self.intake_config = IntakeConfig(
            roller_id=42,
            beam_break_id=3,
            pivot_motor_id=31,
            gear_ratio=16 / 510,
            pivot_ff=FFConstants(0.19, 0, 0, 0.41),
            pivot_pid=PIDConstants(5.3, 0, 0.07),
            profile_constants=ProfileConstants(0, 0),  # TODO: probably implement these
            CANbus=self.CANbus,
        )

        self.shooter_config = ShooterConfig(
            indexer_id=47,
            beam_break_id=1,
            pivot_motor_id=35,
            bottom_flywheel_motor_id=36,
            top_flywheel_motor_id=37,
            shooter_speed_ff=FFConstants(0.0862775, 0.113191/4, 0, 0),
            shooter_gear_ratio=12 / 15,
            pivot_abs_encoder_id=0,
            pivot_pid=PIDConstants(20, 5, 0),
            pivot_profile_constraints=ProfileConstants(
                9999, 1000
            ),  # TODO: probably implement these
            pivot_ff=FFConstants(0, 0.03, 0, 0),
            pivot_gear_ratio=1 / 108,
            CANbus=self.CANbus,
        )

        self.mech = Mechanism2d(4, 4, Color8Bit(255, 255, 255))
        self.intake_mech_root = self.mech.getRoot("Intake", 0, 1.5)
        self.shooter_mech_root = self.mech.getRoot("Shooter", 0, 3)

        SmartDashboard.putData("Mechanism", self.mech)

        self.timer = Timer()
        self._set_up_notifications()

    def disabledInit(self) -> None:
        IO.flush_publishers()  # Have to do here because publishers aren't setup otherwise

    def teleopPeriodic(self) -> None:
        if not self.timer.isRunning():
            self.timer.restart()

        with self.consumeExceptions():
            self._drive_with_joystick()

        if self.controller.getXButton():
            self.drivetrain.enable_motion_limiting()
        elif self.drivetrain.is_motion_limited():
            self.drivetrain.disable_motion_limiting()

        if self.controller.getYButtonPressed():
            self.drivetrain.gyro.set_yaw(0)

        if self.controller.getStartButton() or self.aux_controller.getStartButton():
            self.cancel_all()

        if self.aux_controller.getBButton(): 
            self.drivetrain.operator_lock = True
        
        if self.aux_controller.getAButton(): 
            self.drivetrain.operator_lock = False

        if (
            self.controller.getRightTriggerAxis() > 0.2
            and not self.intake.has_note()
            and not self.shooter.has_note()
            and self.intake.state not in [IntakeStates.RETRACTING, IntakeStates.FEEDING]
        ):
            self.intake.grab_note()
        elif (self.intake.state == IntakeStates.DEPLOYED or self.shooter.state == ShooterStates.HOLDING) and not self.intake.has_note():
            self.intake.go_to_idle()
        elif self.intake.state in [IntakeStates.RETRACTING, IntakeStates.FEEDING]:
            self.shooter.feed()
        
        if (
            self.controller.getLeftTriggerAxis() > 0.2 and
            self.shooter.state == ShooterStates.HOLDING and
            self.shooter.has_note()
        ):
            self.shooter.shoot()


    def robotPeriodic(self) -> None:
        # Stops unimportant notifications during comp
        IO.update_publishers()
        self.watchdog.addEpoch("I/O")

        NotificationManager.send_notifications(DriverStation.isFMSAttached())
        self.watchdog.addEpoch("Notification Sending")

        SmartDashboard.updateValues()

    def _drive_with_joystick(self) -> None:
        vx = (
            utils.filter_input(self.controller.getLeftY())
            * self.drivetrain_config.max_translation_speed
        )
        vy = (
            utils.filter_input(self.controller.getLeftX())
            * self.drivetrain_config.max_translation_speed
        )
        omega = (
            utils.filter_input(self.controller.getRightX())
            * self.drivetrain_config.max_rotation_speed
        )

        if self.isReal():
            self.drivetrain.signal = DriveSignal(ChassisSpeeds(-vx, -vy, -omega))
        else:
            self.drivetrain.signal = DriveSignal(ChassisSpeeds(vx, vy, omega))

    def _set_up_notifications(self) -> None:
        NotificationManager.add_conditional_notification(
            Notification(
                level=NotificationLevel.WARNING,
                title="Low Battery Voltage",
                description="Consider swapping batteries soon",
            ),
            lambda: (RobotController.getBatteryVoltage() < 8 and self.isReal())
            or (
                RobotController.getBatteryVoltage() < 12.25
                and self.isDisabled()
                and self.isReal()
            )
            # The sims are harder on battery voltage
            or (RobotController.getBatteryVoltage() < 6 and self.isSimulation()),
        )

        NotificationManager.add_conditional_notification(
            Notification(
                level=NotificationLevel.ERROR,
                title="Robot is browning out",
                description="Stop robot and replace battery",
            ),
            lambda: RobotController.isBrownedOut(),
        )

    def cancel_all(self) -> None: 
        self.intake.eject()
        self.shooter.eject()
