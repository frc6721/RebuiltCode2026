package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.io.HopperIO;
import frc.robot.subsystems.hopper.io.HopperIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Hopper subsystem controls the mechanism that moves game pieces within the robot.
 *
 * <p><b>Hardware:</b> Single NEO motor with a SparkMax controller, duty cycle controlled.
 *
 * <p><b>How It Works:</b> Intentionally simple - just runs forward, backward, or stops. No PID or
 * feedback control needed. Works similarly to the feeder but sits upstream in the game piece path.
 *
 * <p>Uses the AdvantageKit IO layer pattern for hardware abstraction.
 */
public class Hopper extends SubsystemBase {

  private final HopperIO _hopperIO;
  private final HopperIOInputsAutoLogged _hopperInputs = new HopperIOInputsAutoLogged();

  /**
   * Creates a new Hopper subsystem.
   *
   * @param hopperIO The hardware interface for hopper control
   */
  public Hopper(HopperIO hopperIO) {
    this._hopperIO = hopperIO;
  }

  /**
   * Periodic method called every 20ms. Reads and logs sensor data (motor voltage, current) to
   * AdvantageKit. No closed-loop control - speed is set directly by commands.
   */
  @Override
  public void periodic() {
    // Update and log sensor inputs from hardware
    _hopperIO.updateInputs(_hopperInputs);
    Logger.processInputs("Hopper", _hopperInputs);
  }

  /**
   * Sets the hopper motor speed using open-loop duty cycle control.
   *
   * @param speed Duty cycle from -1.0 (full reverse) to +1.0 (full forward)
   */
  public void setHopperSpeed(double speed) {
    _hopperIO.setMotorSpeed(speed);
  }

  /** Stops the hopper motor. Equivalent to {@code setHopperSpeed(0.0)}. */
  public void stop() {
    _hopperIO.setMotorSpeed(0.0);
  }
}
