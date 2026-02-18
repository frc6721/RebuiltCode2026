package frc.robot.subsystems.feeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.feeder.FeederConstants;

/**
 * Simulation implementation of FeederIO using two SparkMax motors in leader-follower configuration.
 *
 * <p>This is a simple simulation without physics - it just provides voltage and current logging for
 * both feeder motors. The feeder doesn't need realistic physics since it's just duty cycle
 * controlled.
 */
public class SimFeederIO implements FeederIO {

  // Simulated motors
  private final SparkMax _leftFeederMotor;
  private final SparkMax _rightFeederMotor;

  // REV simulation wrappers
  private final SparkMaxSim _leftFeederSim;
  private final SparkMaxSim _rightFeederSim;

  // Control state
  private double _dutyCycle = 0.0;

  /**
   * Creates a new SimFeederIO with two simulated motors in leader-follower configuration.
   *
   * <p>Initializes simulated motors using constants from FeederConstants.
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
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Calculate simulated values based on duty cycle
    double appliedVoltage = _dutyCycle * RoboRioSim.getVInVoltage();

    // Approximate current draw based on duty cycle (split between two motors)
    double simulatedCurrentPerMotor = Math.abs(_dutyCycle) * 7.5; // ~15A total at full power

    // Update battery simulation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simulatedCurrentPerMotor * 2.0));

    // Update both SparkMax simulations
    _leftFeederSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _leftFeederSim.iterate(
        _dutyCycle * 5000.0, // Approximate RPM at full speed
        RoboRioSim.getVInVoltage(),
        0.02);

    _rightFeederSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _rightFeederSim.iterate(_dutyCycle * 5000.0, RoboRioSim.getVInVoltage(), 0.02);

    // Populate inputs - left motor (leader)
    inputs._leftFeederMotorTemperature = Celsius.of(35.0);
    inputs._leftFeederMotorVoltage = Volts.of(appliedVoltage);
    inputs._leftFeederMotorCurrent = Amps.of(simulatedCurrentPerMotor);

    // Right motor (follower)
    inputs._rightFeederMotorTemperature = Celsius.of(35.0);
    inputs._rightFeederMotorVoltage = Volts.of(appliedVoltage);
    inputs._rightFeederMotorCurrent = Amps.of(simulatedCurrentPerMotor);
  }

  @Override
  public void setMotorSpeed(double speed) {
    _dutyCycle = Math.max(-1.0, Math.min(1.0, speed));
    _leftFeederMotor.set(_dutyCycle);
  }
}
