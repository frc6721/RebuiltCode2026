package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeConstants;

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

  public static Command resetIntakeEncoder(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.resetEncoder();
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

  // public static Command runUntilCurrentSpike(Intake intake){
  //   return Commands.startEnd(() ->
  //   {

  //   }, intake.setLinearMotorVoltage(Volts.of(0)), intake);
  // }

  /**
   * Creates a command to home the intake. Use when you need to reset the 0 position of the linear
   * slide motor controller
   */
  public static Command homeIntake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.homeIntake();
        },
        intake);
  }

  // ==================== COMPOSITION COMMANDS ====================

  /**
   * Extends the intake and runs rollers to acquire game pieces. Sets the position once (PID handles
   * movement) and turns on rollers — finishes immediately so other commands can run.
   *
   * <p>Use with {@code whileTrue()} or pair with a trigger to stop rollers when done.
   *
   * @param intake The intake subsystem
   * @return A command that extends and starts acquiring
   */
  public static Command acquireGamePiece(Intake intake) {
    return setIntakeGoalPosition(intake, IntakePosition.EXTENDED)
        .andThen(runIntakeRollers(intake))
        .withName("AcquireGamePiece");
  }

  /**
   * Extends the intake and continuously runs rollers to acquire game pieces. Unlike {@link
   * #acquireGamePiece}, this command runs continuously every 20ms so it maintains the intake
   * subsystem requirement for the full duration.
   *
   * <p>Use with {@code whileTrue()} — rollers run as long as the button is held. The intake
   * position is set to EXTENDED on the first loop and the PID maintains it.
   *
   * @param intake The intake subsystem
   * @return A command that extends and continuously runs rollers until interrupted
   */
  public static Command acquireGamePieceContinuous(Intake intake) {
    return Commands.run(
            () -> {
              intake.setIntakePosition(IntakePosition.EXTENDED);
              intake.turnOnIntakeRollers();
            },
            intake)
        .withName("AcquireGamePieceContinuous");
  }

  /**
   * Retracts the intake and stops the rollers. Stows the intake safely inside the frame perimeter.
   *
   * @param intake The intake subsystem
   * @return A command that retracts and stops rollers
   */
  public static Command stowIntake(Intake intake) {
    return setIntakeGoalPosition(intake, IntakePosition.RETRACTED)
        .andThen(stopIntakeRollers(intake))
        .withName("StowIntake");
  }

  /**
   * Runs the intake rollers at the spit voltage to eject game pieces.
   *
   * @param intake The intake subsystem
   * @return A command that runs the rollers in reverse to spit
   */
  public static Command spitIntakeRollers(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setRollerVoltage(Volts.of(IntakeConstants.Roller.SPIT_SPEED.get()));
        },
        intake);
  }

  /**
   * Jostles the hopper fuel by running the intake rollers at a slow speed. Use to help settle game
   * pieces before shooting.
   *
   * @param intake The intake subsystem
   * @return A repeating jostle command that runs until interrupted
   */
  public static Command jostleIntake(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setRollerVoltage(Volts.of(IntakeConstants.Roller.SLOW_ACQUIRE_SPEED.get()));
            },
            intake)
        .andThen(
            Commands.sequence(
                    setIntakeGoalPosition(intake, IntakePosition.JOSTLE_EXTENDED),
                    Commands.waitSeconds(IntakeConstants.JOSTLE_HALF_CYCLE_DURATION_SECONDS.get()),
                    setIntakeGoalPosition(intake, IntakePosition.JOSTLE_RETRACTED),
                    Commands.waitSeconds(IntakeConstants.JOSTLE_HALF_CYCLE_DURATION_SECONDS.get()))
                .repeatedly())
        .withName("JostleIntake")
        .finallyDo(
            () -> {
              intake.stopRollers();
              intake.setIntakePosition(IntakePosition.EXTENDED);
            });

    // return Commands.sequence(
    //         setIntakeGoalPosition(intake, IntakePosition.JOSTLE_EXTENDED),
    //         Commands.waitSeconds(IntakeConstants.JOSTLE_HALF_CYCLE_DURATION_SECONDS.get()),
    //         setIntakeGoalPosition(intake, IntakePosition.JOSTLE_RETRACTED),
    //         Commands.waitSeconds(IntakeConstants.JOSTLE_HALF_CYCLE_DURATION_SECONDS.get()))
    //     .repeatedly()
    //     .withName("JostleIntake");
  }
}
