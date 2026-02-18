package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intake.IntakeConstants;

public class RealIntakeIO implements IntakeIO {
  private SparkMax _rightPivotMotor;
  private SparkMax _leftPivotMotor;
  private SparkMax _rollerMotor;
  private AbsoluteEncoder _pivotEncoder;

  public RealIntakeIO() {
    configPivotMotors();
    configRollerMotor();
  }

  public void configPivotMotors() {
    // Configure right (leader) pivot motor
    _rightPivotMotor =
        new SparkMax(HardwareConstants.CanIds.RIGHT_PIVOT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .inverted(IntakeConstants.Hardware.RIGHT_PIVOT_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.PIVOT_SMART)
        .secondaryCurrentLimit(IntakeConstants.CurrentLimits.PIVOT_SECONDARY)
        .voltageCompensation(12.0);

    // Configure absolute encoder on right motor
    rightConfig
        .absoluteEncoder
        .inverted(IntakeConstants.Hardware.PIVOT_ENCODER_INVERTED)
        .positionConversionFactor(IntakeConstants.Hardware.PIVOT_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(IntakeConstants.Hardware.PIVOT_ENCODER_VELOCITY_FACTOR)
        .averageDepth(2);

    tryUntilOk(
        _rightPivotMotor,
        5,
        () ->
            _rightPivotMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    _pivotEncoder = _rightPivotMotor.getAbsoluteEncoder();

    // Configure left (follower) pivot motor
    _leftPivotMotor =
        new SparkMax(HardwareConstants.CanIds.LEFT_PIVOT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.PIVOT_SMART)
        .secondaryCurrentLimit(IntakeConstants.CurrentLimits.PIVOT_SECONDARY)
        .voltageCompensation(12.0);

    // Configure left motor to follow right motor
    leftConfig.follow(_rightPivotMotor, true);

    tryUntilOk(
        _leftPivotMotor,
        5,
        () ->
            _leftPivotMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void configRollerMotor() {
    _rollerMotor = new SparkMax(HardwareConstants.CanIds.ROLLER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(IntakeConstants.Hardware.ROLLER_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.ROLLER_SMART)
        .secondaryCurrentLimit(IntakeConstants.CurrentLimits.ROLLER_SECONDARY)
        .voltageCompensation(12.0);

    tryUntilOk(
        _rollerMotor,
        5,
        () ->
            _rollerMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    // Right pivot motor
    ifOk(
        _rightPivotMotor,
        _rightPivotMotor::getMotorTemperature,
        (value) -> inputs._intakeRightPivotMotorTemperature = Celsius.of(value));
    ifOk(
        _rightPivotMotor,
        _pivotEncoder::getVelocity,
        (value) -> inputs._intakeRightPivotMotorVelocity = RadiansPerSecond.of(value));
    ifOk(
        _rightPivotMotor,
        _pivotEncoder::getPosition,
        (value) ->
            inputs._intakeRightPivotMotorPosition =
                new Rotation2d(value).minus(IntakeConstants.Hardware.PIVOT_ZERO_ROTATION));
    ifOk(
        _rightPivotMotor,
        new java.util.function.DoubleSupplier[] {
          _rightPivotMotor::getAppliedOutput, _rightPivotMotor::getBusVoltage
        },
        (values) -> inputs._intakeRightPivotMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _rightPivotMotor,
        _rightPivotMotor::getOutputCurrent,
        (value) -> inputs._intakeRightPivotMotorCurrent = Amps.of(value));

    // Left pivot motor
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor::getMotorTemperature,
        (value) -> inputs._intakeLeftPivotMotorTemperature = Celsius.of(value));
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor.getEncoder()::getVelocity,
        (value) -> inputs._intakeLeftPivotMotorVelocity = RadiansPerSecond.of(value));
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor.getEncoder()::getPosition,
        (value) -> inputs._intakeLeftPivotMotorPosition = new Rotation2d(value));
    ifOk(
        _leftPivotMotor,
        new java.util.function.DoubleSupplier[] {
          _leftPivotMotor::getAppliedOutput, _leftPivotMotor::getBusVoltage
        },
        (values) -> inputs._intakeLeftPivotMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor::getOutputCurrent,
        (value) -> inputs._intakeLeftPivotMotorCurrent = Amps.of(value));

    // Roller motor
    ifOk(
        _rollerMotor,
        _rollerMotor::getMotorTemperature,
        (value) -> inputs._intakeRollerMotorTemperature = Celsius.of(value));
    ifOk(
        _rollerMotor,
        _rollerMotor.getEncoder()::getVelocity,
        (value) -> inputs._intakeRollerMotorVelocity = RadiansPerSecond.of(value));
    ifOk(
        _rollerMotor,
        new java.util.function.DoubleSupplier[] {
          _rollerMotor::getAppliedOutput, _rollerMotor::getBusVoltage
        },
        (values) -> inputs._intakeRollerMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _rollerMotor,
        _rollerMotor::getOutputCurrent,
        (value) -> inputs._intakeRollerMotorCurrent = Amps.of(value));
  }

  // Pivot motor methods
  public void setPivotMotorVoltage(double volts) {
    _rightPivotMotor.setVoltage(volts);
  }

  public void setIntakePivotDutyCycleOutput(double output) {
    _rightPivotMotor.set(output);
  }

  // Roller motor methods
  public void setRollerMotorOutput(double output) {
    _rollerMotor.set(output);
  }
}
