package frc.robot.subsystems.feeder.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    // Left feeder motor (leader)
    public Temperature _leftFeederMotorTemperature;
    public AngularVelocity _leftFeederMotorVelocity;
    public Voltage _leftFeederMotorVoltage;
    public Current _leftFeederMotorCurrent;

    // Right feeder motor (follower)
    public Temperature _rightFeederMotorTemperature;
    public AngularVelocity _rightFeederMotorVelocity;
    public Voltage _rightFeederMotorVoltage;
    public Current _rightFeederMotorCurrent;

    // High-frequency samples from the 100Hz odometry thread.
    // Each loop the thread collects up to 2 samples (100Hz / 50Hz loop = 2 per loop).
    // The timestamps and velocities arrays are always the same length.
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryVelocitiesRPM = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /**
   * Sets the feeder motor to a target velocity using closed-loop PID control with feedforward. The
   * motor controller handles maintaining the speed using its on-board PID + Motion Magic profiling.
   *
   * @param speed Target angular velocity (use RPM.of(), RadiansPerSecond.of(), etc.)
   */
  public default void setFeederVelocity(AngularVelocity speed) {}

  /**
   * Sets the feeder motor voltage directly. Used for feedforward characterization routines where
   * you need precise voltage control.
   *
   * @param voltage The voltage to apply to the motor
   */
  public default void setMotorVoltage(Voltage voltage) {}

  /**
   * Stops the feeder motor and resets the PID integral accumulator to prevent integral windup on
   * the next start. Matches the pattern used by ShooterIO.stopFlywheel().
   */
  public default void stopFeeder() {}
}
