from magicbot import MagicRobot
from ntcore import NetworkTableInstance
from phoenix6 import CANBus
from wpilib import (
    AddressableLED,
    Color8Bit,
    DataLogManager,
    DriverStation,
    Mechanism2d,
    RobotController,
    SmartDashboard,
    Timer,
    XboxController,
    SendableChooser,
)
from wpilib.deployinfo import getDeployData
from wpimath.kinematics import ChassisSpeeds

# Components
from components.climber import Climber
from components.leds import LEDController, LEDMode
from components.drivetrain import DriveSignal, Drivetrain
from components.modules.generic_talon_fx_module import GenericTalonFXModule

from utilities import helpers as utils
from utilities.configs import DrivetrainConfig, SwerveConfig
from utilities.elasticlib import Notification, NotificationLevel, NotificationManager
from utilities.helpers import FFConstants, MotorTypes, PIDConstants
from utilities.IO_helpers import IO

class MyRobot(MagicRobot):
    drivetrain: Drivetrain
    climber: Climber
    led_control: LEDController  # MagicBot injects this

    def createObjects(self) -> None:
        DataLogManager.start()
        
        # --- Metadata Setup ---
        meta_table = NetworkTableInstance.getDefault().getTable("Metadata")
        deploy_info = getDeployData()
        if deploy_info:
            for key, value in deploy_info.items():
                meta_table.putString(key, value)

        # --- Dashboard Setup ---
        # Tab 1: Mode Selection
        self.mode_chooser = SendableChooser()
        for mode in LEDMode:
            self.mode_chooser.addOption(mode.name, mode)
        self.mode_chooser.setDefaultOption(LEDMode.PULSE.name, LEDMode.PULSE)
        SmartDashboard.putData("LED/Mode Selector", self.mode_chooser)

        # Tab 2: Manual Tuning (Only active in Disabled)
        SmartDashboard.putNumber("LED/Manual/Red", 255)
        SmartDashboard.putNumber("LED/Manual/Green", 0)
        SmartDashboard.putNumber("LED/Manual/Blue", 0)
        SmartDashboard.putNumber("LED/Manual/BPM", 45)
        SmartDashboard.putNumber("LED/Manual/Tail", 5)

        self.controller = XboxController(0)
        self.CANbus = CANBus("*")
        self.led_strip = AddressableLED(0) 

        # --- Drivetrain Config ---
        swerve_config = SwerveConfig(
            drive_ratio=1 / 7.7142857,
            steer_ratio=1 / 7.7142857 if self.isReal() else 1 / 25.9,
            steer_pid_constants=PIDConstants(0, 0, 0.0),
            drive_pid_constants=PIDConstants(0, 0, 0),
            ff_constants=FFConstants(0, 0, 0),
            wheel_radius=0.08592 / 2,
            drive_motor_type=MotorTypes.FALCON_500,
        )

        self.drivetrain_config = DrivetrainConfig(
            drive_motor_ids=(20, 22, 24, 26),
            steer_motor_ids=(21, 23, 25, 27),
            steer_encoder_ids=(19, 18, 17, 16),
            gyro_id=51,
            module_offsets=(46.85, 0.5, 261.21, 57.04),
            drive_inverted=(False, False, False, False),
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

        self.timer = Timer()
        self._last_pov = -1

    def disabledInit(self) -> None:
        IO.flush_publishers()
        
    def disabledPeriodic(self) -> None:
        """
        Dashboard Control Mode:
        When disabled, the robot reads from the SmartDashboard tabs.
        """
        self.led_control.mode = self.mode_chooser.getSelected()
        
        # Pull manual colors/speed from Dashboard Tab 2
        r = int(SmartDashboard.getNumber("LED/Manual/Red", 255))
        g = int(SmartDashboard.getNumber("LED/Manual/Green", 0))
        b = int(SmartDashboard.getNumber("LED/Manual/Blue", 0))
        self.led_control.color = Color8Bit(r, g, b)
        self.led_control.speed_bpm = SmartDashboard.getNumber("LED/Manual/BPM", 45)
        self.led_control.tail_length = int(SmartDashboard.getNumber("LED/Manual/Tail", 5))
        
        self.led_control.execute()

    def teleopPeriodic(self) -> None:
        if not self.timer.isRunning():
            self.timer.restart()

        self._drive_with_joystick()

        # --- D-Pad Mode Cycling ---
        pov = self.controller.getPOV()
        if pov == 90 and self._last_pov != 90: # Right
            self.led_control.cycle_mode(1)
        elif pov == 270 and self._last_pov != 270: # Left
            self.led_control.cycle_mode(-1)
        self._last_pov = pov

        # A Button: Reset to OFF
        if self.controller.getAButtonPressed():
            self.led_control.mode = LEDMode.OFF

        # --- ORIGINAL PRESETS ---
        # These will override whatever is on the dashboard once teleop starts.
        self.apply_presets()
        
        # Logic is already handled in apply_presets, now just execute
        self.led_control.execute()

    def apply_presets(self) -> None:
        """Your original hard-coded LED configurations."""
        mode = self.led_control.mode

        if mode == LEDMode.METEOR:
            self.led_control.color = Color8Bit(0, 255, 0)
            self.led_control.speed_bpm = 80 
            self.led_control.tail_length = 2 

        elif mode == LEDMode.PULSE:
            self.led_control.color = Color8Bit(255, 0, 0) # Red Heartbeat
            self.led_control.speed_bpm = 45 

        elif mode == LEDMode.RAINBOW:
            self.led_control.speed_bpm = 45 
        
        elif mode == LEDMode.SNAKE:
            self.led_control.color = Color8Bit(255, 0, 0)
            self.led_control.tail_length = 4
            self.led_control.speed_bpm = 30
            
        elif mode == LEDMode.MATRIX:
            self.led_control.speed_bpm = 10 
            self.led_control.tail_length = 12 
            
        elif mode == LEDMode.BOUNCE:
            self.led_control.color = Color8Bit(255, 0, 0) 
            self.led_control.speed_bpm = 40
            self.led_control.tail_length = 10
            
        elif mode == LEDMode.CONFETTI:
            self.led_control.speed_bpm = 60 
            self.led_control.tail_length = 5
            
        elif mode == LEDMode.BREATH:
            self.led_control.color = Color8Bit(255, 0, 255) 
            self.led_control.speed_bpm = 15 
            
        elif mode == LEDMode.SCROLL:
            self.led_control.color = Color8Bit(255, 100, 0) 
            self.led_control.speed_bpm = 25
            
        elif mode == LEDMode.FLAMES:
            self.led_control.speed_bpm = 45

    def robotPeriodic(self) -> None:
        IO._handle_signal_refreshing()
        SmartDashboard.updateValues()

    def _drive_with_joystick(self) -> None:
        vx = utils.filter_input(self.controller.getLeftY()) * self.drivetrain_config.max_translation_speed
        vy = utils.filter_input(self.controller.getLeftX()) * self.drivetrain_config.max_translation_speed
        omega = utils.filter_input(self.controller.getRightX()) * self.drivetrain_config.max_rotation_speed

        if self.isReal():
            self.drivetrain.signal = DriveSignal(ChassisSpeeds(-vx, -vy, -omega))
        else:
            self.drivetrain.signal = DriveSignal(ChassisSpeeds(vx, vy, omega))

if __name__ == "__main__":
    import wpilib
    wpilib.run(MyRobot)