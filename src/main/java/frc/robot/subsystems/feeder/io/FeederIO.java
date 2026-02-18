package frc.robot.subsystems.feeder.io;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    // Feeder motor
    public Temperature _feederMotorTemperature;
    public Voltage _feederMotorVoltage;
    public Current _feederMotorCurrent;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /**
   * Sets the feeder motor speed as a duty cycle percentage.
   *
   * @param speed Duty cycle from -1.0 (full reverse) to 1.0 (full forward)
   */
  public default void setMotorSpeed(double speed) {}
}
