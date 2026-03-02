// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.VirtualHopper;
import frc.lib.fuelSim.FuelSim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.io.FeederIO;
import frc.robot.subsystems.feeder.io.RealFeederIO;
import frc.robot.subsystems.feeder.io.SimFeederIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.io.HopperIO;
import frc.robot.subsystems.hopper.io.RealHopperIO;
import frc.robot.subsystems.hopper.io.SimHopperIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.RealIntakeIO;
import frc.robot.subsystems.intake.io.SimIntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.io.RealShooterIO;
import frc.robot.subsystems.shooter.io.ShooterIO;
import frc.robot.subsystems.shooter.io.SimShooterIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Feeder feeder;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Vision vision;

  // FuelSim instance - created once and passed to subsystems that need it
  private final FuelSim fuelSim = new FuelSim();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                // TODO: Change to GyroIONavX if using a NavX
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        intake = new Intake(new RealIntakeIO());
        shooter = new Shooter(new RealShooterIO(), fuelSim);
        feeder = new Feeder(new RealFeederIO());
        hopper = new Hopper(new RealHopperIO());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // Use simulation IO implementations with physics simulation
        intake = new Intake(new SimIntakeIO());
        shooter = new Shooter(new SimShooterIO(), fuelSim);
        feeder = new Feeder(new SimFeederIO());
        hopper = new Hopper(new SimHopperIO());
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // For replay, use empty IO implementations
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {}, fuelSim);
        feeder = new Feeder(new FeederIO() {});
        hopper = new Hopper(new HopperIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    if (RobotBase.isSimulation()) {
      //   configureSimButtonBindings();
      configureButtonBindings();
    } else {
      configureButtonBindings();
    }

    SmartDashboard.putData(
        "Retract Intake", IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.RETRACTED));
    SmartDashboard.putData(
        "Extend Intake", IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.EXTENDED));

    // Configure FuelSim for game piece visualization
    configureFuelSim();
  }

  /**
   * Configures the FuelSim system for game piece simulation and visualization.
   *
   * <p>FuelSim allows us to visualize:
   *
   * <ul>
   *   <li>Game pieces (fuel) on the field
   *   <li>Intake collecting fuel when driven over
   *   <li>Shooter trajectory prediction
   *   <li>Launched fuel following realistic physics
   * </ul>
   *
   * <p>This method:
   *
   * <ol>
   *   <li>Spawns starting fuel on the field
   *   <li>Registers robot dimensions for collision detection
   *   <li>Registers intake bounding box for fuel collection
   *   <li>Adds a dashboard button to reset fuel positions
   *   <li>Starts the simulation
   * </ol>
   */
  private void configureFuelSim() {
    try {
      // Spawn initial fuel on the field (neutral zone and depots)
      fuelSim.spawnStartingFuel();

      // Register robot with FuelSim for collision detection
      // Robot pushes fuel out of the way when driving
      fuelSim.registerRobot(
          Constants.Dimensions.ROBOT_WIDTH.in(Meters),
          Constants.Dimensions.ROBOT_LENGTH.in(Meters),
          Constants.Dimensions.BUMPER_HEIGHT.in(Meters),
          drive::getPose,
          drive::getFieldRelativeSpeeds);

      // Register intake with FuelSim
      // Fuel inside the bounding box will be "collected" when canIntakeFuel() returns true.
      // X is robot-forward: the box runs from the front bumper edge outward by LENGTH.
      // Y is robot-left: centered on the robot's Y centerline by half-WIDTH on each side.
      fuelSim.registerIntake(
          Constants.Dimensions.ROBOT_WIDTH.div(2).in(Meters), // xMin: front bumper edge
          Constants.Dimensions.ROBOT_WIDTH
              .div(2)
              .plus(IntakeConstants.FuelSim.LENGTH)
              .in(Meters), // xMax: LENGTH beyond front bumper
          -IntakeConstants.FuelSim.WIDTH.div(2).in(Meters), // yMin: half-width right of center
          IntakeConstants.FuelSim.WIDTH.div(2).in(Meters), // yMax: half-width left of center
          intake::canIntakeFuel, // BooleanSupplier - checks if intake can collect
          intake::simIntakeFuel); // Runnable - called when fuel is collected

      // Start the simulation (updateSim must still be called each loop)
      fuelSim.start();

      // Add dashboard button to reset fuel and hopper
      SmartDashboard.putData(
          "Reset Fuel",
          Commands.runOnce(
                  () -> {
                    // Clear all fuel from the field
                    fuelSim.clearFuel();
                    // Respawn fuel in starting positions
                    fuelSim.spawnStartingFuel();
                    // Reset virtual hopper to empty
                    VirtualHopper.getInstance().reset();
                  })
              .withName("Reset Fuel")
              .ignoringDisable(true));

    } catch (Exception e) {
      // Log error but don't crash - FuelSim is for visualization only
      System.err.println("FuelSim initialization failed: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * **************************************************************
     *
     * <p>BUTTON BINDINGS:
     *
     * <p>Right bumber: Run shooter and feeder at fixed voltage for testing. Driving joysticks: Same
     * as default joystick swerve drive D-Pad left: Reset gyro to 0° Left bumper: Run intake rollers
     * while held, stop when released. A button: Run hopper at fixed speed while held, stop when
     * released. X button: Manual control of intake linear slide forward while held, stop when
     * released. Y button: Manual control of intake linear slide in reverse while held, stop when
     * released.
     *
     * <p>****************************************************************
     */

    // Default command, normal field-relative drive
    // real controller
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // sim controller in MAC os
    // drive.setDefaultCommand(
    //  DriveCommands.joystickDrive(
    //    drive,
    //  () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> -(controller.getRightTriggerAxis())));

    // Always run the flywheels a little bit during the match so they can spin up quicker when we
    // need them
    // shooter.setDefaultCommand(ShooterCommands.runFlywheelsAtIdle(shooter));

    // controller
    //     .leftBumper()
    //     .whileTrue(IntakeCommands.setIntakeRollersVoltage(intake, 4.0))
    //     .onFalse(IntakeCommands.stopIntakeRollers(intake));

    // Dynamic shooting with auto-aiming:
    // - Continuously adjusts flywheel speed based on distance to alliance hub
    // - Automatically rotates robot so the BACK (shooter) faces the hub
    // - Driver maintains full control of translation (forward/back, left/right)
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         // Combine auto-aim driving with shooting sequence
    //         // useBackOfRobot = true because the shooter is rear-mounted
    //         DriveCommands.joystickDriveAtAngle(
    //                 drive,
    //                 () -> -controller.getLeftY(),
    //                 () -> -controller.getLeftX(),
    //                 () -> RobotState.getInstance().getAngleToAllianceHub(),
    //                 true) // true = aim back of robot (shooter) at the hub
    //             .alongWith(ShooterCommands.shootToHubSequence(shooter, feeder, hopper)))
    //     .onFalse(
    //
    // FeederCommands.stopFeeder(feeder).andThen(ShooterCommands.runFlywheelsAtIdle(shooter)));

    // controller
    //     .rightBumper()
    //     .onTrue(HopperCommands.runHopperAtPercentOutput(hopper, .5))
    //     .onFalse(HopperCommands.runHopperAtPercentOutput(hopper, 0));

    controller
        .rightBumper()
        .onTrue(
            ShooterCommands.runShooterAndFeederAtVoltage(shooter, feeder, 3.5, 12)
                .andThen(HopperCommands.runHopperAtPercentOutput(hopper, 0.7)))
        .onFalse(
            ShooterCommands.runShooterAndFeederAtVoltage(shooter, feeder, 0, 0)
                .alongWith(HopperCommands.runHopperAtPercentOutput(hopper, 0)));

    // TESTING COMMAND
    // run the shooter and feeder at a fixed voltage
    // update these values to run faster or slower.
    // Max motor power is 12 volts
    controller
        .leftBumper()
        // .whileTrue(ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3500)))
        .onTrue(IntakeCommands.setIntakeRollersVoltage(intake, -6))
        .onFalse(IntakeCommands.setIntakeRollersVoltage(intake, 0));

    // controller
    //     .leftBumper()
    //     .whileTrue(ShooterCommands.feedforwardCharacterization(shooter))
    //     .onFalse(ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(0)));

    /*
     * Run hopper at fixed speed for testing. Adjust speed in command to change speed.
     * NOTE: this is % output between -1 and 1, not voltage. So 0.3 means 30% of max speed.
     */
    // controller
    //     .a()
    //     .whileTrue(HopperCommands.runHopperAtPercentOutput(hopper, .3))
    //     .onFalse(HopperCommands.stopHopper(hopper));

    /*
     * Manual control of intake linear slide for testing. Adjust voltage in command to change speed.
     *
     */
    // controller
    //     .leftBumper()
    //     .onTrue(IntakeCommands.setIntakeLinearVoltage(intake, 3.0))
    //     .onFalse(IntakeCommands.setIntakeLinearVoltage(intake, 0.0));
    controller
        .rightTrigger(0.5)
        .onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.EXTENDED));

    /*
     * Manual control of intake linear slide in reverse for testing. Adjust voltage in command to change speed.
     */
    // controller
    //     .y()
    //     .onTrue(IntakeCommands.setIntakeLinearVoltage(intake, -3.0))
    //     .onFalse(IntakeCommands.setIntakeLinearVoltage(intake, 0.0));
    controller
        .leftTrigger(0.5)
        .onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.RETRACTED));

    // Reset gyro to 0° when left dpad is pressed
    controller
        .pov(270)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // D-Pad up: Teleport the robot to directly in front of the alliance hub
    // Useful for quickly positioning the robot for shooting tests.
    // AllianceFlipUtil (inside RobotState) automatically handles red vs. blue alliance.
    controller
        .pov(0)
        .onTrue(
            Commands.runOnce(
                    () -> drive.setPose(RobotState.getInstance().getPoseInFrontOfAllianceHub()),
                    drive)
                .ignoringDisable(true));

    // controller
    //     .x()
    //     .onTrue(
    //         ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3500)))
    //     .onFalse(
    //         ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(0)));

    // Used for shooter characterization routines. Not for normal use.
    // Original X binding replaced with shooter characterization bindings
    // controller
    //     .x()
    //     .onTrue(Commands.run(() -> shooter.setFlyWheelDutyCycle(0.015), shooter))
    //     .onFalse(Commands.run(() -> shooter.setFlyWheelDutyCycle(0.0), shooter));
  }

  private void configureSimButtonBindings() {
    /**
     * **************************************************************
     *
     * <p>BUTTON BINDINGS:
     *
     * <p>Right bumber: Run shooter and feeder at fixed voltage for testing. Driving joysticks: Same
     * as default joystick swerve drive D-Pad left: Reset gyro to 0° Left bumper: Run intake rollers
     * while held, stop when released. A button: Run hopper at fixed speed while held, stop when
     * released. X button: Manual control of intake linear slide forward while held, stop when
     * released. Y button: Manual control of intake linear slide in reverse while held, stop when
     * released.
     *
     * <p>****************************************************************
     */

    // Default command, normal field-relative drive
    // real controller
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightTriggerAxis()));

    // sim controller in MAC os
    // drive.setDefaultCommand(
    //  DriveCommands.joystickDrive(
    //    drive,
    //  () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> -(controller.getRightTriggerAxis())));

    // Always run the flywheels a little bit during the match so they can spin up quicker when we
    // need them
    // shooter.setDefaultCommand(ShooterCommands.runFlywheelsAtIdle(shooter));

    // controller
    //     .leftBumper()
    //     .whileTrue(IntakeCommands.setIntakeRollersVoltage(intake, 4.0))
    //     .onFalse(IntakeCommands.stopIntakeRollers(intake));

    // Dynamic shooting with auto-aiming:
    // - Continuously adjusts flywheel speed based on distance to alliance hub
    // - Automatically rotates robot so the BACK (shooter) faces the hub
    // - Driver maintains full control of translation (forward/back, left/right)
    controller
        .y()
        .whileTrue(
            // Combine auto-aim driving with shooting sequence
            // useBackOfRobot = true because the shooter is rear-mounted
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> RobotState.getInstance().getAngleToAllianceHub(),
                    true) // true = aim back of robot (shooter) at the hub
                .alongWith(ShooterCommands.shootToHubSequence(shooter, feeder, hopper)))
        .onFalse(
            FeederCommands.stopFeeder(feeder).andThen(ShooterCommands.runFlywheelsAtIdle(shooter)));

    // controller
    //     .rightBumper()
    //     .onTrue(HopperCommands.runHopperAtPercentOutput(hopper, .5))
    //     .onFalse(HopperCommands.runHopperAtPercentOutput(hopper, 0));

    // controller
    //     .x()
    //     .onTrue(
    //         ShooterCommands.runShooterAndFeederAtVoltage(shooter, feeder, 3.5, 12)
    //             .andThen(HopperCommands.runHopperAtPercentOutput(hopper, 0.7)))
    //     .onFalse(
    //         ShooterCommands.runShooterAndFeederAtVoltage(shooter, feeder, 0, 0)
    //             .alongWith(HopperCommands.runHopperAtPercentOutput(hopper, 0)));

    // TESTING COMMAND
    // run the shooter and feeder at a fixed voltage
    // update these values to run faster or slower.
    // Max motor power is 12 volts
    controller
        .leftBumper()
        // .whileTrue(ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3500)))
        .onTrue(IntakeCommands.setIntakeRollersVoltage(intake, -6))
        .onFalse(IntakeCommands.setIntakeRollersVoltage(intake, 0));

    // controller
    //     .leftBumper()
    //     .whileTrue(ShooterCommands.feedforwardCharacterization(shooter))
    //     .onFalse(ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(0)));

    /*
     * Run hopper at fixed speed for testing. Adjust speed in command to change speed.
     * NOTE: this is % output between -1 and 1, not voltage. So 0.3 means 30% of max speed.
     */
    // controller
    //     .a()
    //     .whileTrue(HopperCommands.runHopperAtPercentOutput(hopper, .3))
    //     .onFalse(HopperCommands.stopHopper(hopper));

    /*
     * Manual control of intake linear slide for testing. Adjust voltage in command to change speed.
     *
     */
    // controller
    //     .leftBumper()
    //     .onTrue(IntakeCommands.setIntakeLinearVoltage(intake, 3.0))
    //     .onFalse(IntakeCommands.setIntakeLinearVoltage(intake, 0.0));
    controller
        .rightTrigger(0.5)
        .onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.EXTENDED));

    /*
     * Manual control of intake linear slide in reverse for testing. Adjust voltage in command to change speed.
     */
    // controller
    //     .y()
    //     .onTrue(IntakeCommands.setIntakeLinearVoltage(intake, -3.0))
    //     .onFalse(IntakeCommands.setIntakeLinearVoltage(intake, 0.0));
    controller
        .leftTrigger(0.5)
        .onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.RETRACTED));

    // Reset gyro to 0° when left dpad is pressed
    controller
        .pov(270)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // D-Pad up: Teleport the robot to directly in front of the alliance hub
    // Useful for quickly positioning the robot for shooting tests.
    // AllianceFlipUtil (inside RobotState) automatically handles red vs. blue alliance.
    controller
        .pov(0)
        .onTrue(
            Commands.runOnce(
                    () -> drive.setPose(RobotState.getInstance().getPoseInFrontOfAllianceHub()),
                    drive)
                .ignoringDisable(true));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Returns the FuelSim instance used by this robot container.
   *
   * <p>Used by {@link Robot} to call {@code updateSim()} each loop in simulation.
   *
   * @return the shared FuelSim instance
   */
  public FuelSim getFuelSim() {
    return fuelSim;
  }
}
