import math
from enum import Enum

import wpilib
import wpimath.units as units
from magicbot import tunable

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from rev import SparkMax, SparkBaseConfig

from wpilib import DigitalInput, SmartDashboard, DutyCycleEncoder
from wpimath.controller import ArmFeedforward, ProfiledPIDController, SimpleMotorFeedforwardRadians
from wpimath.filter import SlewRateLimiter
from wpimath.trajectory import TrapezoidProfile

from utilities.configs import ShooterConfig
from utilities.helpers import clamp
from utilities.IO import ShooterIO

FEED_DELAY = 0
FEEDING_ANGLE = 3
# Make sure this value works. It's untested, but it should hold the note at an
HOLDING_ANGLE = 25
# angle that's closer to the angle it will be shot at to save time
INDEXER_FEED_VOLTAGE = -0.25*12
INDEXER_SHOOTING_VOLTAGE = -0.25*12
SHOOTER_SHOOTING_SPEED = 20
# Actual value is probably 53-55, but to be safe, stay lower for now
SUBWOOFER_SHOOTING_ANGLE = 47

MIN_SHOOTER_ANGLE = -7
MAX_SHOOTER_ANGLE = 43

MAX_SHOOTER_SPEED = 80

class ShooterStates(Enum): 
    IDLE = 1
    AIMING = 2
    SHOOTING = 3
    FEEDING = 4
    HOLDING = 5
    EJECT = 6

class Shooter: 
    manual_tuning_mode = tunable(False)

    def __init__(self, config: ShooterConfig): 
        self.beam_break = DigitalInput(config.beam_bread_id)
        self.angle_absolute_encoder = DutyCycleEncoder(config.pivot_abs_encoder_id)
        self.angle_absolute_encoder.setInverted(True)

        self.pivot_configs = TalonFXConfiguration()
        self.pivot_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # self.pivot_configs.current_limits.supply_current_limit = 2 #TODO: Decide if we want to uncomment this and rest of current limiters
        self.pivot_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.pivot_motor = TalonFX(config.pivot_motor_id, config.CANbus)
        self.pivot_motor.configurator.apply(self.pivot_configs)

        self.pivot_pid = ProfiledPIDController(
            config.pivot_pid.p,
            config.pivot_pid.i,
            config.pivot_pid.d,
            TrapezoidProfile.Constraints(config.pivot_profile_constraints.max_vel, 
                                         config.pivot_profile_constraints.max_acc)
        )
        self.pivot_pid.setIZone(0.2)
        self.pivot_pid.setTolerance(0.02)
        
        self.pivot_ff = ArmFeedforward(
            config.pivot_ff.kS,
            config.pivot_ff.kG if config.pivot_ff is not None else 0.0,
            config.pivot_ff.kA,
            config.pivot_ff.kV
        )

        self.shoot_configs = TalonFXConfiguration()
        # self.shoot_configs.current_limits.supply_current_limit = 2
        self.shoot_configs.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        
        self.bottom_shoot_motor = TalonFX(config.bottom_shoot_motor_id, config.CANbus)
        self.bottom_shoot_motor.configurator.apply(self.shoot_configs)

        self.top_shoot_motor = TalonFX(config.top_shoot_motor_id, config.CANbus)
        self.top_shoot_motor.configurator.apply(self.shoot_configs)

        self.shooter_speed_ff = SimpleMotorFeedforwardRadians(
            config.shooter_speed_ff.kS,
            config.shooter_speed_ff.kV,
            config.shooter_speed_ff.kA
        )

        self.indexer_config = SparkBaseConfig().inverted(False).setIdleMode(SparkBaseConfig.IdleMode.kBrake)

        self.indexer = SparkMax(config.indexer_id, SparkMax.MotorType.kBrushless)
        self.indexer.configure(self.indexer_config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)

        self._state = ShooterStates.IDLE

        self.shot_start_time = float('nan')
        self.feed_start_time = float('nan')
        self.shoot_timer = wpilib.Timer()

        self.config = config

        self.io = ShooterIO()
        self._set_up_logging()

        self.pivot_motor.set_position((self.angle_absolute_encoder.get() - 0.176) / self.config.pivot_gear_ratio)
        self.pivot_pid.reset(self.get_pivot_angle())

        '''
        # -2.96 in motor rotations (assumes shooter starts resting against cover)
        self.pivot_motor.set_position(-2.96)

        offset = 0

        for i in range(40):
            offset += (self.shoot_absolute_encoder.get() ) / 40
            time.sleep(0.01)

        offset *= -1
        offset += 176.468/360

        self.pivot_motor.set_position(offset*108)
        self.pivot_pid.reset(self.get_pivot_degrees())
        '''

    @property
    def state(self) -> ShooterStates: 
        return self._state
    
    @state.setter
    def state(self, state: ShooterStates) -> None: 
        if not isinstance(state, ShooterStates): 
            raise ValueError("State must be of type ShooterStates")
        self._state = state

    def go_to_idle(self) -> None: 
        self.state = ShooterStates.IDLE

    def shoot(self) -> None: 
        self.state = ShooterStates.AIMING

    def feed(self) -> None: 
        self.state = ShooterStates.FEEDING

    def eject(self) -> None: 
        self.state = ShooterStates.EJECT

    def has_note(self) -> bool: 
        return not self.beam_break.get()
    
    def get_abs_position(self) -> float: 
        return self.angle_absolute_encoder.get()
    
    def get_pivot_angle(self) -> float: 
        return self.pivot_motor.get_position().value * self.config.pivot_gear_ratio * math.tau
    
    def get_pivot_speed_radians(self) -> float: 
        return self.pivot_motor.get_velocity().value * self.config.pivot_gear_ratio * math.tau
    
    def get_shooter_speed(self) -> float: 
        return self.top_shoot_motor.get_velocity().value_as_double * self.config.shooter_gear_ratio
    
    def at_target_position(self) -> bool: 
        return self.pivot_pid.atGoal()
    
    def at_target_speed(self) -> bool: 
        return self.get_shooter_speed() >=  self.io.target_shooter_speed
    
    def set_target_position(self, target_position) -> None: 
        self.pivot_pid.setGoal(target_position)
    
    def execute(self): 
        #TODO: Implement logic with io for sport mode
        self._handle_state_logic()
            
        self.io.target_shooter_angle = clamp(self.io.target_shooter_angle, math.radians(MIN_SHOOTER_ANGLE), math.radians(MAX_SHOOTER_ANGLE))
        self.io.target_shooter_speed = clamp(self.io.target_shooter_speed, -MAX_SHOOTER_SPEED, MAX_SHOOTER_SPEED)
        self.io.indexer_voltage = clamp(self.io.indexer_voltage, -12, 12)

        if (self.get_pivot_angle() >= math.radians(MIN_SHOOTER_ANGLE)) and (self.get_pivot_angle() <= math.radians(MAX_SHOOTER_ANGLE)): 
            self.pivot_pid.setGoal(self.io.target_shooter_angle)
            target_voltage = self.pivot_pid.calculate(
                self.get_pivot_angle()) + self.pivot_ff.calculate(self.get_pivot_angle(), 0.0)
            self.io.pivot_target_voltage = target_voltage
            self.pivot_motor.set_control(VoltageOut(target_voltage))
            # Add shooter voltage to elastic

            flywheel_voltage = self.shooter_speed_ff.calculate(self.io.target_shooter_speed)
            self.top_shoot_motor.set_control(VoltageOut(flywheel_voltage))
            self.bottom_shoot_motor.set_control(VoltageOut(flywheel_voltage))
            self.indexer.setVoltage(self.io.indexer_voltage)

        elif self.get_pivot_angle() >= math.radians(MAX_SHOOTER_ANGLE): 
            self.pivot_motor.set_control(VoltageOut(0))
            self.top_shoot_motor.set_control(VoltageOut(0))
            self.bottom_shoot_motor.set_control(VoltageOut(0))

        else: 
            self.pivot_motor.set_control(VoltageOut(2))
            self.top_shoot_motor.set_control(VoltageOut(0))
            self.bottom_shoot_motor.set_control(VoltageOut(0))

    def _handle_state_logic(self) -> None: 
        if self.manual_tuning_mode and not self.io.tuning_sendables_sent:
            SmartDashboard.putData("Shooter Pivot PID", self.pivot_pid)
            self.io.tuning_sendables_sent = True
        
        if self.manual_tuning_mode:
            self.io.target_shooter_angle = self.pivot_pid.getGoal().position
            self.io.target_shooter_speed = 0
            self.io.indexer_voltage = 0
            return

        self.io.state = self.state.name

        match self.state: 
            case ShooterStates.IDLE: 
                self.io.target_shooter_angle = math.radians(FEEDING_ANGLE)
                self.io.indexer_voltage = 0
                self.io.target_shooter_speed = 0

            case ShooterStates.FEEDING: 
                self.io.target_shooter_angle = math.radians(FEEDING_ANGLE)
                self.io.indexer_voltage = INDEXER_FEED_VOLTAGE
                self.io.target_shooter_speed = 0

                if self.has_note(): 
                    if math.isnan(self.feed_start_time): 
                        self.feed_start_time = self.shoot_timer.getFPGATimestamp()
                elif self.shoot_timer.getFPGATimestamp()-self.feed_start_time >= FEED_DELAY: 
                    self.state = ShooterStates.HOLDING
                    self.io.indexer_voltage = 0

            case ShooterStates.HOLDING: 
                self.io.target_shooter_angle = math.radians(HOLDING_ANGLE)
                self.io.indexer_voltage = 0
                self.io.target_shooter_speed = 0

            case ShooterStates.AIMING: 
                self.io.target_shooter_angle = math.radians(SUBWOOFER_SHOOTING_ANGLE)
                self.io.indexer_voltage = 0
                self.io.target_shooter_speed = SHOOTER_SHOOTING_SPEED
                if self.at_target_position() and self.at_target_speed(): 
                    self.state = ShooterStates.SHOOTING
            
            case ShooterStates.SHOOTING: 
                self.io.target_shooter_angle = math.radians(SUBWOOFER_SHOOTING_ANGLE)
                self.io.target_shooter_speed = SHOOTER_SHOOTING_SPEED
                self.io.indexer_voltage = INDEXER_SHOOTING_VOLTAGE
                
                if math.isnan(self.shot_start_time): 
                    self.shot_start_time = self.shoot_timer.getFPGATimestamp()
                elif not self.has_note(): 
                    self.state = ShooterStates.IDLE
                    self.shot_start_time = float("nan")
            
            case ShooterStates.EJECT: 
                self.io.target_shooter_angle = 0
                self.io.target_shooter_speed = 50
                
                if self.get_shooter_speed() > 30: 
                    self.io.indexer_voltage = INDEXER_SHOOTING_VOLTAGE
                else: 
                    self.io.indexer_voltage = 0
                
                if math.isnan(self.shot_start_time): 
                    self.shot_start_time = self.shoot_timer.getFPGATimestamp()
                elif self.shoot_timer.getFPGATimestamp()-self.shot_start_time >= 1.2: 
                    self.state = ShooterStates.IDLE
                    self.shot_start_time = float("nan")
            
    
    def _set_up_logging(self) -> None: 
        self.io.add_function(self.has_note)
        self.io.add_function(self.get_pivot_angle, math.degrees)
        self.io.add_function(self.get_pivot_speed_radians)
        self.io.add_function(self.at_target_position)
        self.io.add_function(self.get_shooter_speed)
        self.io.add_function(self.at_target_speed)
        self.io.add_function(self.get_abs_position, lambda x: x - 0.176)

        self.io.target_shooter_angle = 0
        self.io.target_shooter_speed = 0
        self.io.indexer_voltage = 0

        self.pivot_pid.setGoal(self.io.target_shooter_angle) 

        self.io.shooter_angle_supplier = self.pivot_motor.get_position()
        self.io.shooter_angle_velocity_supplier = self.pivot_motor.get_velocity()
        self.io.shooter_velocity_supplier = self.top_shoot_motor.get_velocity()

        BaseStatusSignal.set_update_frequency_for_all(
            250, self.io.shooter_angle_supplier, self.io.shooter_angle_velocity_supplier, self.io.shooter_velocity_supplier
        )
