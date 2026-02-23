package frc.robot.subsystems.feeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederConstants;

/**
 * Real hardware implementation of FeederIO using two SparkMax motor controllers in a
 * leader-follower configuration.
 *
 * <p>The left motor is the leader and the right motor follows it inverted, so both feed wheels
 * physically spin the same direction.
 */
public class RealFeederIO implements FeederIO {
  private final SparkMax _leftFeederMotor;
  private final SparkMax _rightFeederMotor;

  public RealFeederIO() {
    _leftFeederMotor = new SparkMax(Constants.CanIds.FEEDER_LEFT_MOTOR_ID, MotorType.kBrushless);
    _rightFeederMotor = new SparkMax(Constants.CanIds.FEEDER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    configFeederMotors();
  }

  /** Configures both feeder motors. Left is leader, right follows inverted. */
  public void configFeederMotors() {
    // ── Left motor (leader) ──
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .inverted(FeederConstants.Motor.INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.CurrentLimits.SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(FeederConstants.CurrentLimits.SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    tryUntilOk(
        _leftFeederMotor,
        5,
        () ->
            _leftFeederMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // ── Right motor (follower, inverted relative to leader) ──
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.CurrentLimits.SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(FeederConstants.CurrentLimits.SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    // Follow the left motor, inverted so both wheels physically spin the same direction
    rightConfig.follow(_leftFeederMotor, true);

    tryUntilOk(
        _rightFeederMotor,
        5,
        () ->
            _rightFeederMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Left feeder motor (leader)
    inputs._leftFeederMotorTemperature = Celsius.of(_leftFeederMotor.getMotorTemperature());
    inputs._leftFeederMotorVoltage =
        Volts.of(_leftFeederMotor.getAppliedOutput() * _leftFeederMotor.getBusVoltage());
    inputs._leftFeederMotorCurrent = Amps.of(_leftFeederMotor.getOutputCurrent());

    // Right feeder motor (follower)
    inputs._rightFeederMotorTemperature = Celsius.of(_rightFeederMotor.getMotorTemperature());
    inputs._rightFeederMotorVoltage =
        Volts.of(_rightFeederMotor.getAppliedOutput() * _rightFeederMotor.getBusVoltage());
    inputs._rightFeederMotorCurrent = Amps.of(_rightFeederMotor.getOutputCurrent());
  }

  @Override
  public void setMotorSpeed(double speed) {
    // Only set on leader; follower follows automatically
    _leftFeederMotor.set(speed);
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    // Only set on leader; follower follows automatically
    _leftFeederMotor.setVoltage(voltage);
  }
}
