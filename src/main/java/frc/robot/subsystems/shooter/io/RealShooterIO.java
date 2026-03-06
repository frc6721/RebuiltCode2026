package frc.robot.subsystems.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.SparkOdometryThread;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Queue;

/**
 * Real hardware implementation of ShooterIO using two REV SparkFlex motor controllers in a
 * leader-follower configuration.
 *
 * <p>The left motor is the leader and the right motor follows it inverted, so both flywheels
 * physically spin the same direction. The base inversion is controlled by {@link
 * ShooterConstants.Mechanical#INVERTED}.
 */
public class RealShooterIO implements ShooterIO {
  private final SparkFlex _leftFlywheelMotor;
  private final SparkFlex _rightFlywheelMotor;

  // High-frequency queues fed by the 100Hz odometry thread
  private final Queue<Double> _timestampQueue;
  private final Queue<Double> _velocityQueue;

  public RealShooterIO() {
    _leftFlywheelMotor =
        new SparkFlex(Constants.CanIds.FLYWHEEL_LEFT_MOTOR_ID, MotorType.kBrushless);
    _rightFlywheelMotor =
        new SparkFlex(Constants.CanIds.FLYWHEEL_RIGHT_MOTOR_ID, MotorType.kBrushless);

    configFlywheelMotors();

    // Register the left motor's velocity signal with the shared 100Hz odometry thread.
    // This gives us ~2 velocity samples per 20ms robot loop instead of just 1.
    _timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    _velocityQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(_leftFlywheelMotor, _leftFlywheelMotor.getEncoder()::getVelocity);
  }

  /** Configures both flywheel motors. Left is leader, right follows inverted. */
  public void configFlywheelMotors() {
    // ── Left motor (leader) ──
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig
        .inverted(ShooterConstants.Mechanical.INVERTED)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0);

    // Only apply current limits if enabled - disable during testing/characterization
    if (ShooterConstants.CurrentLimits.ENABLE_CURRENT_LIMITS) {
      leftConfig
          .smartCurrentLimit(ShooterConstants.CurrentLimits.SMART)
          .secondaryCurrentLimit(ShooterConstants.CurrentLimits.SECONDARY);
    }
    leftConfig.closedLoop.pid(
        ShooterConstants.getFlywheelKP(),
        ShooterConstants.getFlywheelKI(),
        ShooterConstants.getFlywheelKD());
    leftConfig.closedLoop.maxMotion.maxAcceleration(
        ShooterConstants.Limits.MAX_ACCEL.in(RPM.per(Second)));

    // Configure the encoder velocity signal to publish at the odometry thread frequency.
    // This ensures the SparkOdometryThread can collect fresh samples every 10ms (100Hz).
    leftConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        _leftFlywheelMotor,
        5,
        () ->
            _leftFlywheelMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // ── Right motor (follower, inverted relative to leader) ──
    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);

    // Only apply current limits if enabled - disable during testing/characterization
    if (ShooterConstants.CurrentLimits.ENABLE_CURRENT_LIMITS) {
      rightConfig
          .smartCurrentLimit(ShooterConstants.CurrentLimits.SMART)
          .secondaryCurrentLimit(ShooterConstants.CurrentLimits.SECONDARY);
    }

    // Follow the left motor, inverted so both wheels physically spin the same direction
    rightConfig.follow(_leftFlywheelMotor, true);

    tryUntilOk(
        _rightFlywheelMotor,
        5,
        () ->
            _rightFlywheelMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Left flywheel motor (leader)
    inputs._leftFlywheelMotorTemperature = Celsius.of(_leftFlywheelMotor.getMotorTemperature());
    inputs._leftFlywheelMotorVelocity = RPM.of(_leftFlywheelMotor.getEncoder().getVelocity());
    inputs._leftFlywheelMotorVoltage =
        Volts.of(_leftFlywheelMotor.getAppliedOutput() * _leftFlywheelMotor.getBusVoltage());
    inputs._leftFlywheelMotorCurrent = Amps.of(_leftFlywheelMotor.getOutputCurrent());

    // Right flywheel motor (follower)
    inputs._rightFlywheelMotorTemperature = Celsius.of(_rightFlywheelMotor.getMotorTemperature());
    inputs._rightFlywheelMotorVelocity = RPM.of(_rightFlywheelMotor.getEncoder().getVelocity());
    inputs._rightFlywheelMotorVoltage =
        Volts.of(_rightFlywheelMotor.getAppliedOutput() * _rightFlywheelMotor.getBusVoltage());
    inputs._rightFlywheelMotorCurrent = Amps.of(_rightFlywheelMotor.getOutputCurrent());

    // Drain the high-frequency queues collected by the 100Hz odometry thread.
    // Convert the raw RPM doubles into a primitive array for AdvantageKit logging.
    inputs.odometryTimestamps = _timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryVelocitiesRPM =
        _velocityQueue.stream().mapToDouble(Double::doubleValue).toArray();
    _timestampQueue.clear();
    _velocityQueue.clear();
  }

  // ── Flywheel motor methods (commands go to leader only; follower follows automatically) ──

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    double targetRPM = speed.in(RPM);

    // Manually calculate feedforward voltage: V = kS * sign(velocity) + kV * velocity
    double kS = ShooterConstants.getFlywheelKS();
    double kV = ShooterConstants.getFlywheelKV();
    double ffVolts = kS * Math.signum(targetRPM) + kV * targetRPM;

    _leftFlywheelMotor
        .getClosedLoopController()
        .setReference(
            targetRPM,
            ControlType.kMAXMotionVelocityControl,
            com.revrobotics.spark.ClosedLoopSlot.kSlot0,
            ffVolts,
            com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setFlyWheelDutyCycle(double output) {
    _leftFlywheelMotor.set(output);
  }

  @Override
  public void stopFlywheel() {
    // Reset the integral accumulator to prevent integral windup
    _leftFlywheelMotor.getClosedLoopController().setIAccum(0);
    _leftFlywheelMotor.stopMotor();
  }

  @Override
  public void setFlywheelVoltage(Voltage volts) {
    _leftFlywheelMotor.setVoltage(volts.magnitude());
  }
}
