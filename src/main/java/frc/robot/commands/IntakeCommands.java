package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

/**
 * Factory class for creating intake-related commands using static factory methods. Keeps command
 * creation logic in one place and makes RobotContainer cleaner.
 */
public class IntakeCommands {

  /**
   * Creates a command to set the intake linear slide to a target position. Sets position once and
   * finishes immediately - the PID controller in {@link Intake#periodic()} handles the actual
   * movement.
   *
   * @param intake The intake subsystem
   * @param position The target position (IntakePosition.RETRACTED or IntakePosition.EXTENDED)
   * @return A command that sets the intake target position and finishes immediately
   */
  public static Command setIntakeGoalPosition(Intake intake, IntakePosition position) {
    return Commands.runOnce(
        () -> {
          intake.setIntakePosition(position);
        },
        intake);
  }

  /**
   * Creates a command to stop the intake rollers.
   *
   * @param intake The intake subsystem
   * @return A command that stops the intake rollers
   */
  public static Command stopIntakeRollers(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.stopRollers();
        },
        intake);
  }

  /**
   * Creates a command for manual duty cycle control of the intake linear slide, bypassing PID.
   *
   * <p><b>Warning:</b> For testing/setup only. Use {@link #setIntakeGoalPosition} for normal
   * operation.
   *
   * @param intake The intake subsystem
   * @param output The duty cycle output (-1.0 to +1.0)
   * @return A command that manually controls the intake linear slide
   */
  public static Command setIntakeLinearDutyCycle(Intake intake, double output) {
    return Commands.run(
        () -> {
          intake.setLinearMotorDutyCycleOutput(output);
        },
        intake);
  }

  public static Command setIntakeRollersVoltage(Intake intake, double voltage) {
    return Commands.run(
        () -> {
          intake.setRollerVoltage(Volts.of(voltage));
        },
        intake);
  }

  public static Command setIntakeLinearVoltage(Intake intake, double voltage) {
    return Commands.run(
        () -> {
          intake.setLinearMotorVoltage(Volts.of(voltage));
        },
        intake);
  }

  /**
   * Creates a command to run the intake rollers at {@link
   * frc.robot.subsystems.intake.IntakeConstants.Roller#ACQUIRE_SPEED} for collecting game pieces.
   *
   * @param intake The intake subsystem
   * @return A command that runs the intake rollers
   */
  public static Command runIntakeRollers(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.turnOnIntakeRollers();
        },
        intake);
  }
}
