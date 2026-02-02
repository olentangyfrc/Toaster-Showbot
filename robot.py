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
        
        # Metadata for Advantagescope/Elastic
        meta_table = NetworkTableInstance.getDefault().getTable("Metadata")
        deploy_info = getDeployData()
        if deploy_info:
            for key, value in deploy_info.items():
                meta_table.putString(key, value)

        self.controller = XboxController(0)
        self.CANbus = CANBus("*")

        # LED Hardware: Single strip of 29 LEDs on Port 0
        self.led_strip = AddressableLED(0) 

        # Drivetrain Config
        swerve_config = SwerveConfig(
            drive_ratio=1 / 7.7142857,
            steer_ratio=1 / 7.7142857 if self.isReal() else 1 / 25.9,
            steer_pid_constants=PIDConstants(3.7, 0, 0.05),
            drive_pid_constants=PIDConstants(0.5, 0, 0),
            ff_constants=FFConstants(0.2278, 2.4176, 0),
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

    def teleopPeriodic(self) -> None:
        if not self.timer.isRunning():
            self.timer.restart()

        with self.consumeExceptions():
            self._drive_with_joystick()
        if  self.controller.getXButton():
            self.drivetrain.disable_motion_limiting()
        else: 
            self.drivetrain.enable_motion_limiting()
            
        

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

        # --- LED Mode Configurations ---
        # We only assign values here. The math happens in components/leds.py
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

        # --- Drivetrain Utilities ---
        if self.controller.getYButtonPressed():
            self.drivetrain.gyro.set_yaw(0)

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