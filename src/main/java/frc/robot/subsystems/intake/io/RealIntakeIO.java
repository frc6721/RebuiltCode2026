package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * Real hardware implementation of IntakeIO for a linear slide + roller intake.
 *
 * <p><b>Hardware:</b>
 *
 * <ul>
 *   <li>Linear motor (SparkMax + NEO): Extends/retracts the intake on a linear rail
 *   <li>Roller motor (SparkMax + NEO): Spins rollers to acquire game pieces
 * </ul>
 *
 * <p>Position is tracked using the linear motor's built-in relative encoder. The encoder is zeroed
 * at startup (assumes intake starts fully retracted). Positive rotation = extending out.
 */
public class RealIntakeIO implements IntakeIO {
  private final SparkMax _linearMotor;
  private final SparkMax _rollerMotorLeader;
  private final SparkMax _rollerMotorFollower;

  private final RelativeEncoder _linearEncoder;

  public RealIntakeIO() {
    _linearMotor = new SparkMax(Constants.CanIds.INTAKE_LINEAR_MOTOR_ID, MotorType.kBrushless);
    _rollerMotorLeader =
        new SparkMax(Constants.CanIds.INTAKE_ROLLER_MOTOR_LEADER_ID, MotorType.kBrushless);
    _rollerMotorFollower =
        new SparkMax(Constants.CanIds.INTAKE_ROLLER_MOTOR_FOLLOWER_ID, MotorType.kBrushless);

    configLinearMotor();
    configRollerMotor();

    // Get the internal encoder from the linear motor
    _linearEncoder = _linearMotor.getEncoder();
  }

  /** Configures the linear slide motor with position/velocity conversion factors. */
  public void configLinearMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(IntakeConstants.Hardware.LINEAR_MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.LINEAR_SMART)
        .secondaryCurrentLimit(IntakeConstants.CurrentLimits.LINEAR_SECONDARY)
        .voltageCompensation(12.0);

    // Configure the internal encoder conversion factors
    config
        .encoder
        .positionConversionFactor(IntakeConstants.Mechanical.LINEAR_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(IntakeConstants.Mechanical.LINEAR_VELOCITY_CONVERSION_FACTOR);

    tryUntilOk(
        _linearMotor,
        5,
        () ->
            _linearMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /** Configures the roller motor. */
  public void configRollerMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(IntakeConstants.Hardware.ROLLER_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.ROLLER_SMART)
        .secondaryCurrentLimit(IntakeConstants.CurrentLimits.ROLLER_SECONDARY)
        .voltageCompensation(12.0);

    tryUntilOk(
        _rollerMotorLeader,
        5,
        () ->
            _rollerMotorLeader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // ── Right motor (follower, inverted relative to leader) ──
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.ROLLER_SMART)
        .secondaryCurrentLimit(IntakeConstants.CurrentLimits.ROLLER_SECONDARY)
        .voltageCompensation(12.0);

    // Follow the left motor, inverted so both wheels physically spin the same direction
    followerConfig.follow(_rollerMotorLeader, true);

    tryUntilOk(
        _rollerMotorFollower,
        5,
        () ->
            _rollerMotorFollower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Linear slide motor
    ifOk(
        _linearMotor,
        _linearMotor::getMotorTemperature,
        (value) -> inputs._linearMotorTemperature = Celsius.of(value));
    ifOk(
        _linearMotor,
        _linearEncoder::getVelocity,
        (value) -> inputs._linearMotorVelocity = RadiansPerSecond.of(value));
    ifOk(_linearMotor, _linearEncoder::getPosition, (value) -> inputs._linearMotorPosition = value);
    ifOk(
        _linearMotor,
        new java.util.function.DoubleSupplier[] {
          _linearMotor::getAppliedOutput, _linearMotor::getBusVoltage
        },
        (values) -> inputs._linearMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _linearMotor,
        _linearMotor::getOutputCurrent,
        (value) -> inputs._linearMotorCurrent = Amps.of(value));

    // Roller motor
    ifOk(
        _rollerMotorLeader,
        _rollerMotorLeader::getMotorTemperature,
        (value) -> inputs._rollerMotorTemperature = Celsius.of(value));
    ifOk(
        _rollerMotorLeader,
        _rollerMotorLeader.getEncoder()::getVelocity,
        (value) -> inputs._rollerMotorVelocity = RadiansPerSecond.of(value));
    ifOk(
        _rollerMotorLeader,
        new java.util.function.DoubleSupplier[] {
          _rollerMotorLeader::getAppliedOutput, _rollerMotorLeader::getBusVoltage
        },
        (values) -> inputs._rollerMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _rollerMotorLeader,
        _rollerMotorLeader::getOutputCurrent,
        (value) -> inputs._rollerMotorCurrent = Amps.of(value));
  }

  // Linear slide motor methods
  @Override
  public void setLinearMotorVoltage(Voltage volts) {
    _linearMotor.setVoltage(volts);
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    _rollerMotorLeader.setVoltage(volts);
  }

  @Override
  public void setLinearMotorDutyCycle(double output) {
    _linearMotor.set(output);
  }

  @Override
  public void resetLinearEncoder() {
    _linearEncoder.setPosition(0.0);
  }

  // Roller motor methods
  @Override
  public void setRollerMotorOutput(double output) {
    _rollerMotorLeader.set(output);
  }
}
