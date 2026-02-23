package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.io.FeederIO;
import frc.robot.subsystems.feeder.io.FeederIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Feeder subsystem controls the mechanism that feeds game pieces from the hopper into the
 * shooter.
 *
 * <p><b>Hardware:</b> Single NEO motor with AM Sport Gearbox (4:1), 4" Thrifty Squish Wheels,
 * belt-driven (36T pulleys, 120T 5mm pitch belt), with polycarbonate arc guide.
 *
 * <p><b>How It Works:</b> Intentionally simple - just runs forward, backward, or stops. Waits for
 * shooter to reach target RPM, feeds the game piece, then stops. No PID or feedback control needed.
 *
 * <p>Uses the AdvantageKit IO layer pattern for hardware abstraction.
 */
public class Feeder extends SubsystemBase {

  private final FeederIO _feederIO;
  private final FeederIOInputsAutoLogged _feederInputs = new FeederIOInputsAutoLogged();

  /**
   * Creates a new Feeder subsystem.
   *
   * @param feederIO The hardware interface for feeder control
   */
  public Feeder(FeederIO feederIO) {
    this._feederIO = feederIO;
  }

  /**
   * Periodic method called every 20ms. Reads and logs sensor data (motor voltage, current) to
   * AdvantageKit. No closed-loop control - speed is set directly by commands.
   */
  @Override
  public void periodic() {
    // Update and log sensor inputs from hardware
    _feederIO.updateInputs(_feederInputs);
    Logger.processInputs("Feeder", _feederInputs);
  }

  /**
   * Sets the feeder motor speed using open-loop duty cycle control.
   *
   * <p>Only feed when shooter is at target RPM for consistent shots.
   *
   * @param speed Duty cycle from -1.0 (full reverse) to +1.0 (full forward into shooter)
   */
  public void setFeederSpeed(double speed) {
    _feederIO.setMotorSpeed(speed);
  }

  public void runFeederAtVoltage(Voltage voltage) {
    _feederIO.setMotorVoltage(voltage);
  }

  /** Stops the feeder motor. Equivalent to {@code setFeederSpeed(0.0)}. */
  public void stop() {
    _feederIO.setMotorSpeed(0.0);
  }
}
