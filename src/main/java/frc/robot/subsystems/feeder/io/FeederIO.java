package frc.robot.subsystems.feeder.io;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    // Left feeder motor (leader)
    public Temperature _leftFeederMotorTemperature;
    public Voltage _leftFeederMotorVoltage;
    public Current _leftFeederMotorCurrent;

    // Right feeder motor (follower)
    public Temperature _rightFeederMotorTemperature;
    public Voltage _rightFeederMotorVoltage;
    public Current _rightFeederMotorCurrent;
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
