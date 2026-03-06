package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * Factory class for commands that coordinate multiple game-piece subsystems (intake, hopper,
 * feeder).
 *
 * <p>These commands don't belong in a single subsystem's command file because they require multiple
 * subsystems working together. Keeping them here makes RobotContainer cleaner and makes it easy to
 * reuse them across different button bindings or auto routines.
 */
public class GamePieceCommands {

  /**
   * Ejects all game pieces by extending the intake and running every mechanism in reverse.
   *
   * <p>This command:
   *
   * <ol>
   *   <li>Extends the intake so fuel can exit the robot
   *   <li>Runs the intake rollers in the spit direction to push fuel out
   *   <li>Runs the feeder in reverse to push fuel back toward the intake
   *   <li>Runs the hopper in reverse to push fuel back toward the intake
   * </ol>
   *
   * <p>When the command ends (button released or interrupted), all mechanisms stop cleanly.
   *
   * <p><b>Usage:</b> Bind with {@code whileTrue()} so everything stops when the button is released.
   *
   * @param intake The intake subsystem
   * @param feeder The feeder subsystem
   * @param hopper The hopper subsystem
   * @return A command that ejects fuel from all mechanisms
   */
  public static Command spitFuel(Intake intake, Feeder feeder, Hopper hopper) {
    return Commands.runOnce(() -> intake.setIntakePosition(IntakePosition.EXTENDED), intake)
        .andThen(
            Commands.startEnd(
                () -> {
                  // Run all mechanisms in reverse to push fuel out
                  intake.setRollerVoltage(Volts.of(IntakeConstants.Roller.SPIT_SPEED.get()));
                  feeder.runFeederAtVoltage(FeederConstants.Voltages.SPIT_VOLTAGE);
                  hopper.setHopperSpeed(HopperConstants.Speeds.SPIT_SPEED);
                },
                () -> {
                  // Stop everything cleanly when the command ends
                  intake.stopRollers();
                  feeder.stop();
                  hopper.stop();
                },
                intake,
                feeder,
                hopper))
        .withName("SpitFuel");
  }
}
