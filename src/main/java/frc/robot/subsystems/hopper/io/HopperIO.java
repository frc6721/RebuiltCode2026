package frc.robot.subsystems.hopper.io;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Hopper subsystem.
 *
 * <p>Defines the hardware abstraction layer for a single-motor hopper mechanism. Keeps
 * vendor-specific code (SparkMax, etc.) out of the subsystem class.
 */
public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    // Hopper motor
    public Temperature _hopperMotorTemperature;
    public Voltage _hopperMotorVoltage;
    public Current _hopperMotorCurrent;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HopperIOInputs inputs) {}

  /**
   * Sets the hopper motor speed as a duty cycle percentage.
   *
   * @param speed Duty cycle from -1.0 (full reverse) to 1.0 (full forward)
   */
  public default void setMotorSpeed(double speed) {}
}
