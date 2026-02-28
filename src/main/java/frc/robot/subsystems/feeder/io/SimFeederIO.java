package frc.robot.subsystems.feeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.feeder.FeederConstants;

/**
 * Simulation implementation of FeederIO using two SparkMax motors in leader-follower configuration.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Feeder wheels:</b> Uses WPILib's FlywheelSim for realistic physics
 *   <li><b>Motors:</b> Uses REV SparkMaxSim for realistic motor behavior
 *   <li><b>Velocity Control:</b> Feedforward + proportional feedback for velocity targeting
 *   <li><b>Leader-follower:</b> Right motor follows left motor inverted
 * </ul>
 */
public class SimFeederIO implements FeederIO {

  // Simulated motors
  private final SparkMax _leftFeederMotor;
  private final SparkMax _rightFeederMotor;

  // REV simulation wrappers
  private final SparkMaxSim _leftFeederSim;
  private final SparkMaxSim _rightFeederSim;

  // WPILib physics simulation for the feeder wheels
  private final FlywheelSim _feederPhysicsSim;

  // Control state
  private double _appliedVoltage = 0.0;
  private boolean _velocityControlActive = false;
  private double _targetVelocityRPM = 0.0;

  /**
   * Creates a new SimFeederIO with physics simulation.
   *
   * <p>Initializes simulated motors and the flywheel physics simulation using constants from
   * FeederConstants.
   */
  public SimFeederIO() {
    // Create simulated motors (using arbitrary CAN IDs since they don't matter in sim)
    _leftFeederMotor = new SparkMax(70, MotorType.kBrushless);
    _rightFeederMotor = new SparkMax(71, MotorType.kBrushless);

    // ── Configure the left feeder motor (leader) ──
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .inverted(FeederConstants.Motor.INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.CurrentLimits.SMART_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    // Configure PID for velocity control (using simulation PID values)
    leftConfig.closedLoop.pid(
        FeederConstants.getFeederKP(),
        FeederConstants.getFeederKI(),
        FeederConstants.getFeederKD());

    _leftFeederMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ── Configure the right feeder motor (follower, inverted relative to leader) ──
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
    rightConfig.follow(_leftFeederMotor, true);

    _rightFeederMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create REV simulation wrappers
    _leftFeederSim = new SparkMaxSim(_leftFeederMotor, FeederConstants.Mechanical.MOTOR);
    _rightFeederSim = new SparkMaxSim(_rightFeederMotor, FeederConstants.Mechanical.MOTOR);

    // Create the flywheel physics simulation
    // This models the feeder wheel + belt + gearbox as a spinning mass
    _feederPhysicsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                FeederConstants.Mechanical.MOTOR,
                FeederConstants.Mechanical.MOI.in(KilogramSquareMeters),
                FeederConstants.Mechanical.GEAR_RATIO),
            FeederConstants.Mechanical.MOTOR);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // --- Calculate applied voltage ---
    if (_velocityControlActive) {
      // Manually calculate feedforward: V = kS * sign(velocity) + kV * velocity
      double kS = FeederConstants.getFeederKS();
      double kV = FeederConstants.getFeederKV();
      double ffVolts = kS * Math.signum(_targetVelocityRPM) + kV * _targetVelocityRPM;

      // Add proportional feedback for error correction
      double currentRPM = _feederPhysicsSim.getAngularVelocityRPM();
      double error = _targetVelocityRPM - currentRPM;
      double pVolts = error * FeederConstants.getFeederKP() * 100.0;

      _appliedVoltage = Math.max(-12.0, Math.min(12.0, ffVolts + pVolts));
    }

    // --- Update the flywheel physics simulation ---
    _feederPhysicsSim.setInputVoltage(_appliedVoltage);
    _feederPhysicsSim.update(0.02);

    // Update battery simulation based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(_feederPhysicsSim.getCurrentDrawAmps()));

    // Update both SparkMax simulations with the physics results
    _leftFeederSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _leftFeederSim.iterate(
        _feederPhysicsSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    _rightFeederSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _rightFeederSim.iterate(
        _feederPhysicsSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    // --- Read back the simulated sensor values ---
    // Left feeder motor (leader)
    inputs._leftFeederMotorTemperature = Celsius.of(35.0);
    inputs._leftFeederMotorVelocity = RPM.of(_feederPhysicsSim.getAngularVelocityRPM());
    inputs._leftFeederMotorVoltage = Volts.of(_appliedVoltage);
    inputs._leftFeederMotorCurrent = Amps.of(_feederPhysicsSim.getCurrentDrawAmps() / 2.0);

    // Right feeder motor (follower) - same speed, split current
    inputs._rightFeederMotorTemperature = Celsius.of(35.0);
    inputs._rightFeederMotorVelocity = RPM.of(_feederPhysicsSim.getAngularVelocityRPM());
    inputs._rightFeederMotorVoltage = Volts.of(_appliedVoltage);
    inputs._rightFeederMotorCurrent = Amps.of(_feederPhysicsSim.getCurrentDrawAmps() / 2.0);
  }

  @Override
  public void setFeederVelocity(AngularVelocity speed) {
    _velocityControlActive = true;
    _targetVelocityRPM = speed.in(RPM);
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    _velocityControlActive = false;
    _appliedVoltage = voltage.in(Volts);
  }
}
