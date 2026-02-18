package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating feeder-related commands. The feeder commands are simple since the
 * subsystem just feeds game pieces, stops, or runs at a specific speed.
 *
 * <p>Used in shooting sequences: shooter spins up → wait for speed → feed → stop.
 */
public class FeederCommands {
  /** Deadband threshold for joystick input - values below this are treated as zero. */
  private static final double DEADBAND = 0.1;

  /**
   * Creates a command to run the feeder using joystick input with deadband applied. Useful for
   * manual testing.
   *
   * @param feeder The feeder subsystem
   * @param speedSupplier Joystick value supplier (-1.0 to 1.0)
   * @return A command that continuously runs the feeder based on joystick input
   */
  public static Command runFeederWithJoystick(Feeder feeder, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband to joystick input (similar to DriveCommands)
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);

          // Set the motor speed (duty cycle from -1.0 to 1.0)
          feeder.setFeederSpeed(speed);
        },
        feeder);
  }

  /**
   * Creates a command to run the feeder at a fixed duty cycle. Most common way to control the
   * feeder in shooting sequences.
   *
   * <p>Start with lower speeds (0.5-0.7) and increase if needed. Too fast can cause jams.
   *
   * @param feeder The feeder subsystem
   * @param speed Fixed duty cycle from -1.0 (full reverse) to +1.0 (full forward into shooter)
   * @return A command that runs the feeder at the specified speed
   */
  public static Command runFeederAtPercentOutput(Feeder feeder, double speed) {
    return Commands.run(
        () -> {
          feeder.setFeederSpeed(speed);
        },
        feeder);
  }

  /**
   * Creates a command to stop the feeder motor. Uses runOnce() - executes once and finishes
   * immediately.
   *
   * @param feeder The feeder subsystem
   * @return A command that stops the feeder once and finishes
   */
  public static Command stopFeeder(Feeder feeder) {
    return Commands.runOnce(() -> feeder.stop(), feeder);
  }
}
