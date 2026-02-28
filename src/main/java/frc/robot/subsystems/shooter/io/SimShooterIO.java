package frc.robot.subsystems.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Simulation implementation of ShooterIO using two SparkFlex motors in leader-follower
 * configuration.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Flywheel:</b> Uses WPILib's FlywheelSim for realistic flywheel physics
 *   <li><b>Motors:</b> Uses REV SparkFlexSim for realistic motor behavior
 *   <li><b>Velocity Control:</b> On-controller PID for velocity targeting
 *   <li><b>Leader-follower:</b> Right motor follows left motor inverted
 * </ul>
 */
public class SimShooterIO implements ShooterIO {

  // Simulated motors
  private final SparkFlex _leftFlywheelMotor;
  private final SparkFlex _rightFlywheelMotor;

  // REV simulation wrappers
  private final SparkFlexSim _leftFlywheelSim;
  private final SparkFlexSim _rightFlywheelSim;

  // WPILib physics simulation for the flywheel
  private final FlywheelSim _flywheelPhysicsSim;

  // Control state
  private double _appliedVoltage = 0.0;
  private boolean _velocityControlActive = false;
  private double _targetVelocityRPM = 0.0;

  /**
   * Creates a new SimShooterIO with physics simulation.
   *
   * <p>Initializes simulated motors and the flywheel physics simulation using constants from
   * ShooterConstants.
   */
  public SimShooterIO() {
    // Create simulated motors (using arbitrary CAN IDs since they don't matter in sim)
    _leftFlywheelMotor = new SparkFlex(60, MotorType.kBrushless);
    _rightFlywheelMotor = new SparkFlex(61, MotorType.kBrushless);

    // ── Configure the left flywheel motor (leader) ──
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig
        .inverted(ShooterConstants.Mechanical.INVERTED)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CurrentLimits.SMART)
        .voltageCompensation(12.0);

    // Configure PID for velocity control (using simulation PID values)
    leftConfig.closedLoop.pidf(
        ShooterConstants.getFlywheelKP(),
        ShooterConstants.getFlywheelKI(),
        ShooterConstants.getFlywheelKD(),
        ShooterConstants.getFlywheelFF());

    _leftFlywheelMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ── Configure the right flywheel motor (follower, inverted relative to leader) ──
    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);
    rightConfig.follow(_leftFlywheelMotor, true);

    _rightFlywheelMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create REV simulation wrappers
    _leftFlywheelSim = new SparkFlexSim(_leftFlywheelMotor, ShooterConstants.Mechanical.MOTOR);
    _rightFlywheelSim = new SparkFlexSim(_rightFlywheelMotor, ShooterConstants.Mechanical.MOTOR);

    // Create the flywheel physics simulation
    _flywheelPhysicsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                ShooterConstants.Mechanical.MOTOR,
                ShooterConstants.Mechanical.MOI.in(KilogramSquareMeters),
                ShooterConstants.Mechanical.GEAR_RATIO),
            ShooterConstants.Mechanical.MOTOR);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // --- Calculate applied voltage ---
    if (_velocityControlActive) {
      // Manually calculate feedforward: V = kS * sign(velocity) + kV * velocity
      double kS = ShooterConstants.getFlywheelKS();
      double kV = ShooterConstants.getFlywheelKV();
      double ffVolts = kS * Math.signum(_targetVelocityRPM) + kV * _targetVelocityRPM;

      // Add proportional feedback for error correction
      double currentRPM = _flywheelPhysicsSim.getAngularVelocityRPM();
      double error = _targetVelocityRPM - currentRPM;
      double pVolts = error * ShooterConstants.getFlywheelKP() * 100.0;

      _appliedVoltage = Math.max(-12.0, Math.min(12.0, ffVolts + pVolts));
    }

    // --- Update the flywheel physics simulation ---
    _flywheelPhysicsSim.setInputVoltage(_appliedVoltage);
    _flywheelPhysicsSim.update(0.02);

    // Update battery simulation based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(_flywheelPhysicsSim.getCurrentDrawAmps()));

    // Update both SparkFlex simulations with the physics results
    _leftFlywheelSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _leftFlywheelSim.iterate(
        _flywheelPhysicsSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    _rightFlywheelSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _rightFlywheelSim.iterate(
        _flywheelPhysicsSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    // --- Read back the simulated sensor values ---
    // Left flywheel motor (leader)
    inputs._leftFlywheelMotorTemperature = Celsius.of(45.0);
    inputs._leftFlywheelMotorVelocity = RPM.of(_flywheelPhysicsSim.getAngularVelocityRPM());
    inputs._leftFlywheelMotorVoltage = Volts.of(_appliedVoltage);
    inputs._leftFlywheelMotorCurrent = Amps.of(_flywheelPhysicsSim.getCurrentDrawAmps() / 2.0);

    // Right flywheel motor (follower) - same speed, split current
    inputs._rightFlywheelMotorTemperature = Celsius.of(45.0);
    inputs._rightFlywheelMotorVelocity = RPM.of(_flywheelPhysicsSim.getAngularVelocityRPM());
    inputs._rightFlywheelMotorVoltage = Volts.of(_appliedVoltage);
    inputs._rightFlywheelMotorCurrent = Amps.of(_flywheelPhysicsSim.getCurrentDrawAmps() / 2.0);
  }

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    _velocityControlActive = true;
    _targetVelocityRPM = speed.in(RPM);

    // Also set through the actual motor controller for completeness
    _leftFlywheelMotor
        .getClosedLoopController()
        .setReference(_targetVelocityRPM, ControlType.kVelocity);
  }

  @Override
  public void setFlyWheelDutyCycle(double output) {
    _velocityControlActive = false;
    _appliedVoltage = output * 12.0;
    _leftFlywheelMotor.set(output);
  }

  @Override
  public void stopFlywheel() {
    _velocityControlActive = false;
    _targetVelocityRPM = 0.0;
    _appliedVoltage = 0.0;
    _leftFlywheelMotor.stopMotor();
  }

  @Override
  public void setFlywheelVoltage(edu.wpi.first.units.measure.Voltage volts) {
    _velocityControlActive = false;
    _appliedVoltage = volts.magnitude();
  }
}
