package frc.robot.subsystems.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Simulation implementation of ShooterIO that uses physics simulation for realistic flywheel
 * behavior.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Flywheel:</b> Uses WPILib's FlywheelSim for realistic flywheel physics
 *   <li><b>Motor:</b> Uses REV SparkMaxSim for realistic motor behavior
 *   <li><b>Velocity Control:</b> On-controller PID for velocity targeting
 * </ul>
 *
 * <p>The simulation includes:
 *
 * <ul>
 *   <li>Moment of inertia from ShooterConstants
 *   <li>Gear ratio effects
 *   <li>Battery voltage simulation
 *   <li>Realistic spin-up and spin-down behavior
 * </ul>
 */
public class SimShooterIO implements ShooterIO {

  // Simulated motor
  private final SparkMax _flywheelMotor;

  // REV simulation wrapper
  private final SparkMaxSim _flywheelSim;

  // WPILib physics simulation for the flywheel
  private final FlywheelSim _flywheelPhysicsSim;

  // Control state
  private double _appliedVoltage = 0.0;
  private boolean _velocityControlActive = false;
  private double _targetVelocityRPM = 0.0;

  /**
   * Creates a new SimShooterIO with physics simulation.
   *
   * <p>Initializes simulated motor and the flywheel physics simulation using constants from
   * ShooterConstants.
   */
  public SimShooterIO() {
    // Create simulated motor (using arbitrary CAN ID since it doesn't matter in sim)
    _flywheelMotor = new SparkMax(60, MotorType.kBrushless);

    // Configure the flywheel motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ShooterConstants.Mechanical.INVERTED)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CurrentLimits.SMART)
        .voltageCompensation(12.0);

    // Configure PID for velocity control (using simulation PID values)
    config.closedLoop.pidf(
        ShooterConstants.getFlywheelKP(),
        ShooterConstants.getFlywheelKI(),
        ShooterConstants.getFlywheelKD(),
        ShooterConstants.getFlywheelFF());

    _flywheelMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create REV simulation wrapper
    _flywheelSim = new SparkMaxSim(_flywheelMotor, ShooterConstants.Mechanical.MOTOR);

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

    // Set the input voltage to the simulation
    _flywheelPhysicsSim.setInputVoltage(_appliedVoltage);

    // Step the simulation forward (20ms = 0.02s)
    _flywheelPhysicsSim.update(0.02);

    // Update battery simulation based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(_flywheelPhysicsSim.getCurrentDrawAmps()));

    // Update the SparkMax simulation with the physics results
    _flywheelSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _flywheelSim.iterate(
        _flywheelPhysicsSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    // --- Read back the simulated sensor values ---

    inputs._flywheelMotorTemperature = Celsius.of(45.0); // Simulated constant temp
    inputs._flywheelMotorVelocity =
        RotationsPerSecond.of(_flywheelPhysicsSim.getAngularVelocityRPM() / 60.0);
    inputs._flywheelMotorVoltage = Volts.of(_appliedVoltage);
    inputs._flywheelMotorCurrent = Amps.of(_flywheelPhysicsSim.getCurrentDrawAmps());
  }

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    _velocityControlActive = true;
    _targetVelocityRPM = speed.in(RotationsPerSecond) * 60.0;

    // Also set through the actual motor controller for completeness
    _flywheelMotor
        .getClosedLoopController()
        .setReference(_targetVelocityRPM, ControlType.kVelocity);
  }

  @Override
  public void setFlyWheelDutyCycle(double output) {
    _velocityControlActive = false;
    _appliedVoltage = output * 12.0;
    _flywheelMotor.set(output);
  }

  @Override
  public void stopFlywheel() {
    _velocityControlActive = false;
    _targetVelocityRPM = 0.0;
    _appliedVoltage = 0.0;
    _flywheelMotor.stopMotor();
  }

  @Override
  public void setFlywheelVoltage(edu.wpi.first.units.measure.Voltage volts) {
    _velocityControlActive = false;
    _appliedVoltage = volts.magnitude();
  }
}
