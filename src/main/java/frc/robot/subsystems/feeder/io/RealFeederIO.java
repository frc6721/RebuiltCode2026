package frc.robot.subsystems.feeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.SparkOdometryThread;
import frc.robot.subsystems.feeder.FeederConstants;
import java.util.Queue;

/**
 * Real hardware implementation of FeederIO using two SparkMax motor controllers in a
 * leader-follower configuration.
 *
 * <p>The left motor is the leader and the right motor follows it inverted, so both feed wheels
 * physically spin the same direction.
 *
 * <p><b>Velocity Control:</b> Uses the SparkMax's on-board PID controller with Motion Magic
 * (MAXMotion) for smooth acceleration profiling. Feedforward (kS + kV * RPM) is calculated in code
 * and passed as an arbitrary feedforward voltage to the motor controller.
 */
public class RealFeederIO implements FeederIO {
  private final SparkMax _leftFeederMotor;
  private final SparkMax _rightFeederMotor;

  // High-frequency queues fed by the 100Hz odometry thread
  private final Queue<Double> _timestampQueue;
  private final Queue<Double> _velocityQueue;

  public RealFeederIO() {
    _leftFeederMotor = new SparkMax(Constants.CanIds.FEEDER_LEFT_MOTOR_ID, MotorType.kBrushless);
    _rightFeederMotor = new SparkMax(Constants.CanIds.FEEDER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    configFeederMotors();

    // Register the left motor's velocity signal with the shared 100Hz odometry thread.
    // This gives us ~2 velocity samples per 20ms robot loop instead of just 1.
    _timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    _velocityQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(_leftFeederMotor, _leftFeederMotor.getEncoder()::getVelocity);
  }

  /**
   * Configures both feeder motors. Left is leader with PID + Motion Magic, right follows inverted.
   */
  public void configFeederMotors() {
    // ── Left motor (leader) ──
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .inverted(FeederConstants.Motor.INVERTED)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0);

    // Only apply current limits if enabled - disable during testing/characterization
    if (FeederConstants.CurrentLimits.ENABLE_CURRENT_LIMITS) {
      leftConfig
          .smartCurrentLimit(FeederConstants.CurrentLimits.SMART_CURRENT_LIMIT)
          .secondaryCurrentLimit(FeederConstants.CurrentLimits.SECONDARY_CURRENT_LIMIT);
    }

    // Configure PID for velocity control on the motor controller
    leftConfig.closedLoop.pid(
        FeederConstants.getFeederKP(),
        FeederConstants.getFeederKI(),
        FeederConstants.getFeederKD());

    // Configure Motion Magic (MAXMotion) acceleration limit for smooth ramp-up
    leftConfig.closedLoop.maxMotion.maxAcceleration(
        FeederConstants.Limits.MAX_ACCEL.in(RPM.per(Second)));

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
        _leftFeederMotor,
        5,
        () ->
            _leftFeederMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // ── Right motor (follower, inverted relative to leader) ──
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0);

    // Only apply current limits if enabled - disable during testing/characterization
    if (FeederConstants.CurrentLimits.ENABLE_CURRENT_LIMITS) {
      rightConfig
          .smartCurrentLimit(FeederConstants.CurrentLimits.SMART_CURRENT_LIMIT)
          .secondaryCurrentLimit(FeederConstants.CurrentLimits.SECONDARY_CURRENT_LIMIT);
    }

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
    inputs._leftFeederMotorVelocity = RPM.of(_leftFeederMotor.getEncoder().getVelocity());
    inputs._leftFeederMotorVoltage =
        Volts.of(_leftFeederMotor.getAppliedOutput() * _leftFeederMotor.getBusVoltage());
    inputs._leftFeederMotorCurrent = Amps.of(_leftFeederMotor.getOutputCurrent());

    // Right feeder motor (follower)
    inputs._rightFeederMotorTemperature = Celsius.of(_rightFeederMotor.getMotorTemperature());
    inputs._rightFeederMotorVelocity = RPM.of(_rightFeederMotor.getEncoder().getVelocity());
    inputs._rightFeederMotorVoltage =
        Volts.of(_rightFeederMotor.getAppliedOutput() * _rightFeederMotor.getBusVoltage());
    inputs._rightFeederMotorCurrent = Amps.of(_rightFeederMotor.getOutputCurrent());

    // Drain the high-frequency queues collected by the 100Hz odometry thread.
    // Convert the raw RPM doubles into a primitive array for AdvantageKit logging.
    inputs.odometryTimestamps = _timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryVelocitiesRPM =
        _velocityQueue.stream().mapToDouble(Double::doubleValue).toArray();
    _timestampQueue.clear();
    _velocityQueue.clear();
  }

  /**
   * Sets the feeder to a target velocity using the SparkMax's on-board PID + Motion Magic.
   *
   * <p>Feedforward is calculated manually: V = kS * sign(velocity) + kV * velocity This is passed
   * as an arbitrary feedforward voltage alongside the PID reference.
   */
  @Override
  public void setFeederVelocity(AngularVelocity speed) {
    double targetRPM = speed.in(RPM);

    // Manually calculate feedforward voltage: V = kS * sign(velocity) + kV * velocity
    double kS = FeederConstants.getFeederKS();
    double kV = FeederConstants.getFeederKV();
    double ffVolts = kS * Math.signum(targetRPM) + kV * targetRPM;

    // Send the velocity setpoint to the motor controller with Motion Magic profiling
    // The PID handles error correction, the arbFF provides the baseline voltage
    _leftFeederMotor
        .getClosedLoopController()
        .setReference(
            targetRPM,
            ControlType.kMAXMotionVelocityControl,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    // Only set on leader; follower follows automatically
    _leftFeederMotor.setVoltage(voltage);
  }

  @Override
  public void stopFeeder() {
    // Reset the integral accumulator to prevent integral windup on next start
    _leftFeederMotor.getClosedLoopController().setIAccum(0);
    _leftFeederMotor.stopMotor();
  }
}
