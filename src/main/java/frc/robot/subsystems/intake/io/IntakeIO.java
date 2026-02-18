package frc.robot.subsystems.intake.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Intake subsystem.
 *
 * <p>The intake has two motors:
 *
 * <ul>
 *   <li><b>Linear motor:</b> Extends/retracts the intake on a linear slide. Position is tracked via
 *       the motor's internal encoder (0 = fully retracted, positive = extending).
 *   <li><b>Roller motor:</b> Spins rollers to acquire game pieces.
 * </ul>
 */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // Linear slide motor
    public Temperature _linearMotorTemperature;
    public AngularVelocity _linearMotorVelocity;
    public double _linearMotorPosition = 0.0; // Output rotations (0 = retracted, + = extending)
    public Voltage _linearMotorVoltage;
    public Current _linearMotorCurrent;

    // Roller motor
    public Temperature _rollerMotorTemperature;
    public AngularVelocity _rollerMotorVelocity;
    public Voltage _rollerMotorVoltage;
    public Current _rollerMotorCurrent;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  // Linear slide motor methods

  /**
   * Sets the linear motor voltage directly.
   *
   * @param volts Voltage to apply (-12.0 to +12.0)
   */
  public default void setLinearMotorVoltage(double volts) {}

  /**
   * Sets the linear motor duty cycle output directly.
   *
   * @param output Duty cycle from -1.0 to +1.0
   */
  public default void setLinearMotorDutyCycle(double output) {}

  /** Resets the linear motor's internal encoder to zero (current position becomes 0). */
  public default void resetLinearEncoder() {}

  // Roller motor methods

  /**
   * Sets the roller motor output.
   *
   * @param output Duty cycle from -1.0 to +1.0
   */
  public default void setRollerMotorOutput(double output) {}
}
