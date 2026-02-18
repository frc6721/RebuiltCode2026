package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating hopper-related commands. The hopper commands are simple since the
 * subsystem just moves game pieces forward, backward, or stops.
 *
 * <p>Mirrors the feeder command pattern for consistency.
 */
public class HopperCommands {
  /** Deadband threshold for joystick input - values below this are treated as zero. */
  private static final double DEADBAND = 0.1;

  /**
   * Creates a command to run the hopper using joystick input with deadband applied. Useful for
   * manual testing.
   *
   * @param hopper The hopper subsystem
   * @param speedSupplier Joystick value supplier (-1.0 to 1.0)
   * @return A command that continuously runs the hopper based on joystick input
   */
  public static Command runHopperWithJoystick(Hopper hopper, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband to joystick input
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          hopper.setHopperSpeed(speed);
        },
        hopper);
  }

  /**
   * Creates a command to run the hopper at a fixed duty cycle.
   *
   * @param hopper The hopper subsystem
   * @param speed Fixed duty cycle from -1.0 (full reverse) to +1.0 (full forward)
   * @return A command that runs the hopper at the specified speed
   */
  public static Command runHopperAtPercentOutput(Hopper hopper, double speed) {
    return Commands.run(
        () -> {
          hopper.setHopperSpeed(speed);
        },
        hopper);
  }

  /**
   * Creates a command to stop the hopper motor. Uses runOnce() - executes once and finishes
   * immediately.
   *
   * @param hopper The hopper subsystem
   * @return A command that stops the hopper once and finishes
   */
  public static Command stopHopper(Hopper hopper) {
    return Commands.runOnce(() -> hopper.stop(), hopper);
  }
}
