package frc.robot.subsystems.hopper.io;

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
import frc.robot.subsystems.hopper.HopperConstants;

/**
 * Simulation implementation of HopperIO.
 *
 * <p>Simple simulation without physics - provides voltage and current logging for the hopper motor.
 * The hopper doesn't need realistic physics since it's just duty cycle controlled.
 */
public class SimHopperIO implements HopperIO {

  // Simulated motor
  private final SparkMax _hopperMotor;

  // REV simulation wrapper
  private final SparkMaxSim _hopperSim;

  // Control state
  private double _dutyCycle = 0.0;

  /**
   * Creates a new SimHopperIO.
   *
   * <p>Initializes a simulated SparkMax motor using constants from HopperConstants.
   */
  public SimHopperIO() {
    // Create simulated motor (using arbitrary CAN ID since it doesn't matter in sim)
    _hopperMotor = new SparkMax(72, MotorType.kBrushless);

    // Configure the hopper motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(HopperConstants.Motor.INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HopperConstants.CurrentLimits.SMART_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    _hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create REV simulation wrapper
    _hopperSim = new SparkMaxSim(_hopperMotor, HopperConstants.Mechanical.MOTOR);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // Calculate simulated values based on duty cycle
    double appliedVoltage = _dutyCycle * RoboRioSim.getVInVoltage();

    // Approximate current draw based on duty cycle
    double simulatedCurrent = Math.abs(_dutyCycle) * 15.0; // ~15A at full power

    // Update battery simulation
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simulatedCurrent));

    // Update the SparkMax simulation
    _hopperSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _hopperSim.iterate(
        _dutyCycle * 5000.0, // Approximate RPM at full speed
        RoboRioSim.getVInVoltage(),
        0.02);

    // Populate inputs
    inputs._hopperMotorTemperature = Celsius.of(35.0); // Simulated constant temp
    inputs._hopperMotorVoltage = Volts.of(appliedVoltage);
    inputs._hopperMotorCurrent = Amps.of(simulatedCurrent);
  }

  @Override
  public void setMotorSpeed(double speed) {
    _dutyCycle = Math.max(-1.0, Math.min(1.0, speed));
    _hopperMotor.set(_dutyCycle);
  }
}
