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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AllianceFlipUtil;
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
import org.littletonrobotics.junction.Logger;
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

  // ── Auto Preview & Starting Pose Check ──────────────────────────────────────
  // Field2d widget to show the selected auto's path and the robot's current position.
  // Used during disabled/pre-match to verify the robot is placed correctly.
  public final Field2d autoPreviewField = new Field2d();

  // Stores the starting pose of the currently selected auto.
  // Updated when the auto chooser selection changes.
  private Pose2d autoStartPose = new Pose2d();

  // ── Starting Pose Tolerances ────────────────────────────────────────────────
  // How close (in inches) the robot needs to be to the auto's starting position
  // for us to consider it "close enough" to start the match.
  private static final Distance STARTING_POSE_DRIVE_TOLERANCE = Inches.of(6.0);

  // How close (in degrees) the robot's heading needs to be to the auto's starting heading.
  private static final double STARTING_POSE_ROT_TOLERANCE_DEGREES = 5.0;

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

    // Put the auto preview Field2d on the dashboard so we can see the selected path
    SmartDashboard.putData("Auto Preview", autoPreviewField);

    // Configure the button bindings
    if (DriverStation.isFMSAttached()) {
      // Running in competition, only configure real button bindings
      configureButtonBindings();
    } else if (RobotBase.isSimulation()) {
      configureSimButtonBindings();
      //   configureButtonBindings();
      // Configure FuelSim for game piece visualization
      configureFuelSim();
    } else {
      configureButtonBindings();
      // Configure FuelSim for game piece visualization
      configureFuelSim();
    }

    SmartDashboard.putData("Reset Intake Encoder", IntakeCommands.resetIntakeEncoder(intake));
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
     * <p>Right bumper: Run shooter and feeder at fixed voltage for testing. Driving joysticks: Same
     * as default joystick swerve drive D-Pad left: Reset gyro to 0° Left bumper: Run intake rollers
     * while held, stop when released. A button: Snap to nearest straight X-axis heading while held.
     * B button: Point intake at alliance wall while held. X button: Manual control of intake linear
     * slide forward while held, stop when released. Y button: Manual control of intake linear slide
     * in reverse while held, stop when released.
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
    // - Automatically determines target based on field position (hub, feed left, feed right)
    // - Continuously adjusts flywheel speed based on distance to the active target
    // - Automatically rotates robot so the BACK (shooter) faces the active target
    // - Waits until BOTH flywheel is at speed AND robot is facing the target before feeding
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
    //                 () -> RobotState.getInstance().getAngleToActiveTarget(),
    //                 true) // true = aim back of robot (shooter) at the active target
    //             .alongWith(ShooterCommands.shootToActiveTargetSequence(shooter, feeder, hopper)))
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

    // A button: Snap to nearest straight X-axis heading (0° or 180°) while held.
    // Useful for straightening out to drive through the trench.
    // The robot picks whichever direction (intake or shooter) is already closest to the X axis.
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveSnapToNearestXHeading(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // B button: Point the intake (front) at the alliance wall while held.
    // On blue alliance the intake faces 180° (toward blue wall).
    // On red alliance the intake faces 0° (toward red wall).
    // Uses AllianceFlipUtil so the correct heading is chosen automatically.
    controller
        .b()
        .whileTrue(
            DriveCommands.joystickDriveIntakeAtAllianceWall(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

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
     * <p>Right bumper: Run shooter and feeder at fixed voltage for testing. Driving joysticks: Same
     * as default joystick swerve drive D-Pad left: Reset gyro to 0° Left bumper: Run intake rollers
     * while held, stop when released. A button: Snap to nearest straight X-axis heading while held.
     * B button: Point intake at alliance wall while held. X button: Manual control of intake linear
     * slide forward while held, stop when released. Y button: Manual control of intake linear slide
     * in reverse while held, stop when released.
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
    // - Automatically determines target based on field position (hub, feed left, feed right)
    // - Continuously adjusts flywheel speed based on distance to the active target
    // - Automatically rotates robot so the BACK (shooter) faces the active target
    // - Waits until BOTH flywheel is at speed AND robot is facing the target before feeding
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
                    () -> RobotState.getInstance().getAngleToActiveTarget(),
                    true) // true = aim back of robot (shooter) at the active target
                .alongWith(ShooterCommands.shootToActiveTargetSequence(shooter, feeder, hopper)))
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

    // A button: Snap to nearest straight X-axis heading (0° or 180°) while held.
    // Useful for straightening out to drive through the trench.
    // The robot picks whichever direction (intake or shooter) is already closest to the X axis.
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveSnapToNearestXHeading(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // B button: Point the intake (front) at the alliance wall while held.
    // On blue alliance the intake faces 180° (toward blue wall).
    // On red alliance the intake faces 0° (toward red wall).
    // Uses AllianceFlipUtil so the correct heading is chosen automatically.
    controller
        .b()
        .whileTrue(
            DriveCommands.joystickDriveIntakeAtAllianceWall(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

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

  // ── Auto Preview & Starting Pose Check Methods ────────────────────────────

  /** Tracks the last auto name so we only reload paths when the selection changes. */
  private String lastAutoName = "";

  /**
   * Updates the auto path preview on the Field2d when the selected auto changes.
   *
   * <p>Call this periodically (e.g., from {@code disabledPeriodic} or {@code robotPeriodic}). It
   * reads the currently selected auto command's name, loads all paths from that auto file, and
   * draws them on the "Auto Preview" Field2d widget.
   */
  public void updateAutoPreview() {
    try {
      Command selectedAuto = autoChooser.get();
      String autoName = (selectedAuto != null) ? selectedAuto.getName() : "";

      // Only reload when the selection changes
      if (autoName.equals(lastAutoName)) return;
      lastAutoName = autoName;

      // If the selected auto is empty or "none", clear the preview
      if (autoName.isEmpty() || autoName.equalsIgnoreCase("none")) {
        autoPreviewField.getObject("path").setPoses();
        autoPreviewField.getObject("startPose").setPoses();
        return;
      }

      // Load all paths from the selected auto file
      var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      if (paths.isEmpty()) return;

      // Collect all poses from every path, flipping for alliance
      var allPoses = new java.util.ArrayList<Pose2d>();
      for (PathPlannerPath path : paths) {
        for (Pose2d pose : path.getPathPoses()) {
          allPoses.add(AllianceFlipUtil.apply(pose));
        }
      }
      if (allPoses.isEmpty()) return;

      // Get the auto's actual starting pose and use it as the first point
      Pose2d startingPose = new PathPlannerAuto(autoName).getStartingPose();
      if (startingPose != null) {
        allPoses.set(0, AllianceFlipUtil.apply(startingPose));
        autoStartPose = AllianceFlipUtil.apply(startingPose);
      }

      // Draw the path and starting pose on the Field2d
      autoPreviewField.getObject("path").setPoses(allPoses.toArray(new Pose2d[0]));
      autoPreviewField.getObject("startPose").setPose(autoStartPose);

    } catch (Exception e) {
      // If loading fails (e.g., auto file not found), clear the preview
      autoPreviewField.getObject("path").setPoses();
      autoPreviewField.getObject("startPose").setPoses();
    }
  }

  /**
   * Checks and displays the robot's starting pose accuracy relative to the selected autonomous
   * path.
   *
   * <p>This method should be called periodically while the robot is disabled so the drive team can
   * verify the robot is placed correctly before a match. It publishes:
   *
   * <ul>
   *   <li>The robot's current pose on the auto preview Field2d
   *   <li>Distance (in inches) from the auto's starting position
   *   <li>Rotation difference (in degrees) from the auto's starting heading
   *   <li>Boolean flags indicating if position and rotation are within tolerance
   * </ul>
   */
  public void checkStartPose() {
    // Show the robot's current pose on the auto preview field
    autoPreviewField.setRobotPose(drive.getPose());

    try {
      // Get the first pose from the path preview (this is the starting pose)
      Pose2d startPose = autoPreviewField.getObject("path").getPoses().get(0);
      Logger.recordOutput("Auto/StartPose", startPose);

      // Show the starting pose marker on the field
      autoPreviewField.getObject("startPose").setPose(startPose);

      // Calculate distance from robot to starting position
      Distance distanceFromStartPose =
          Meters.of(drive.getPose().getTranslation().getDistance(startPose.getTranslation()));

      // Calculate rotation difference
      double degreesFromStartPose =
          Math.abs(drive.getPose().getRotation().minus(startPose.getRotation()).getDegrees());

      // Publish starting pose info as an array [x, y, degrees]
      double[] startPoseArray = {
        startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees()
      };
      SmartDashboard.putNumberArray("Start Pose (x, y, degrees)", startPoseArray);

      // Publish distance from start (rounded to 2 decimal places)
      SmartDashboard.putNumber(
          "Auto Pose Check/Inches from Start",
          (int) Math.round(distanceFromStartPose.in(Inches) * 100.0) / 100.0);

      // Publish whether the robot is close enough to the starting position
      SmartDashboard.putBoolean(
          "Auto Pose Check/Robot Position Within Tolerance",
          distanceFromStartPose.in(Inches) < STARTING_POSE_DRIVE_TOLERANCE.in(Inches));

      // Publish rotation difference (rounded to 2 decimal places)
      SmartDashboard.putNumber(
          "Auto Pose Check/Degrees from Start",
          (int) Math.round(degreesFromStartPose * 100.0) / 100.0);

      // Publish whether the robot's heading is close enough
      SmartDashboard.putBoolean(
          "Auto Pose Check/Robot Rotation Within Tolerance",
          degreesFromStartPose < STARTING_POSE_ROT_TOLERANCE_DEGREES);

    } catch (Exception e) {
      // No valid path selected — show default "not ready" values
      autoStartPose = drive.getPose();
      SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
      SmartDashboard.putBoolean("Auto Pose Check/Robot Position Within Tolerance", false);
      SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
      SmartDashboard.putBoolean("Auto Pose Check/Robot Rotation Within Tolerance", false);
    }
  }
}
