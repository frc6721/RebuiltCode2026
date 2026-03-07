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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.GamePieceCommands;
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
 *
 * <p><b>Section Order:</b>
 *
 * <ol>
 *   <li>Fields (subsystems, controllers, dashboard)
 *   <li>Constructor (subsystem instantiation, auto setup)
 *   <li>Registered Commands (PathPlanner named commands and event markers)
 *   <li>Button Bindings (configureButtonBindings)
 *   <li>Helper Methods (auto preview, FuelSim, getters)
 * </ol>
 */
public class RobotContainer {

  // ==================== SUBSYSTEMS ====================
  private final Drive drive;
  private final Intake intake;
  private final Feeder feeder;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Vision vision;

  // ==================== SIMULATION ====================
  /** FuelSim instance — created once and passed to subsystems that need it. */
  private final FuelSim fuelSim = new FuelSim();

  // ==================== CONTROLLERS ====================
  private final CommandXboxController controller = new CommandXboxController(0);

  // ==================== DASHBOARD ====================
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

  // ==================== CONSTRUCTOR ====================

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // ── Instantiate subsystems based on robot mode ────────────────────────────
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
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
                new VisionIOLimelight(VisionConstants.camera0Name, (() -> drive.getRotation())),
                new VisionIOLimelight(VisionConstants.camera1Name, (() -> drive.getRotation())));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new SimIntakeIO());
        shooter = new Shooter(new SimShooterIO(), fuelSim);
        feeder = new Feeder(new SimFeederIO());
        hopper = new Hopper(new SimHopperIO());
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot — disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {}, fuelSim);
        feeder = new Feeder(new FeederIO() {});
        hopper = new Hopper(new HopperIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // ── Register PathPlanner commands (MUST happen before building autos) ─────
    registerNamedCommands();
    registerEventMarkers();

    // ── Auto chooser ─────────────────────────────────────────────────────────
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData("Auto Preview", autoPreviewField);

    // ── Button bindings ──────────────────────────────────────────────────────
    configureButtonBindings();

    // ── FuelSim (sim/practice only, not during FMS matches) ──────────────────
    if (!DriverStation.isFMSAttached()) {
      configureFuelSim();
    }

    // ── Dashboard utilities ──────────────────────────────────────────────────
    SmartDashboard.putData("Reset Intake Encoder", IntakeCommands.resetIntakeEncoder(intake));
  }

  // ==================== REGISTERED COMMANDS (PathPlanner) ====================

  /** Registers event marker triggers for PathPlanner path following. */
  private void registerEventMarkers() {
    new EventTrigger("deploy-intake").onTrue(IntakeCommands.acquireGamePiece(intake));

    new EventTrigger("retract-intake").onTrue(IntakeCommands.stowIntake(intake));
  }

  /**
   * Registers named commands for use in PathPlanner autonomous routines.
   *
   * <p><b>Important:</b> This must be called BEFORE creating any PathPlanner autos or paths. The
   * string names used here must exactly match the event marker names in the PathPlanner GUI.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("deploy-intake", IntakeCommands.acquireGamePiece(intake));

    NamedCommands.registerCommand("retract-intake", IntakeCommands.stowIntake(intake));

    // Full auto-aim + shoot sequence for autonomous:
    // 1. Rotate the robot so the BACK (rear-mounted shooter) faces the alliance hub
    // 2. Continuously spin up the flywheel based on distance to the hub
    // 3. Wait until flywheel is at speed AND robot is facing the target (timeout safety)
    // 4. Feed the game piece through the feeder and hopper
    // 5. When done, stop the feeder, hopper, and flywheels
    NamedCommands.registerCommand(
        "shoot-fuel",
        DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0.0,
                () -> 0.0,
                () -> RobotState.getInstance().getAngleToAllianceHub(),
                true)
            .alongWith(ShooterCommands.shootToHubSequence(shooter, feeder, hopper))
            .withTimeout(5.0)
            .finallyDo(
                () -> {
                  feeder.stop();
                  hopper.stop();
                  shooter.stopFlywheels();
                }));

    // Emergency stop for all shooting mechanisms
    NamedCommands.registerCommand(
        "stop-shooter",
        Commands.parallel(
            ShooterCommands.stopFlywheels(shooter),
            FeederCommands.stopFeeder(feeder),
            HopperCommands.stopHopper(hopper)));
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

  // ==================== BUTTON BINDINGS ====================

  /**
   * Configures controller button → command mappings for competition.
   *
   * <pre>
   * ┌─────────────────────────────────────────────────────────────────┐
   * │  LEFT STICK:  Translation (field-relative driving)             │
   * │  RIGHT STICK: Rotation (field-relative driving)                │
   * │                                                                │
   * │  RIGHT TRIGGER: Acquire game piece (extend + rollers) — hold   │
   * │  LEFT TRIGGER:  Spit fuel (reverse all mechanisms) — hold      │
   * │                                                                │
   * │  A BUTTON: Retract intake                                      │
   * │  B BUTTON: Extend intake                                       │
   * │  X BUTTON: (unassigned — reserved for climber)                 │
   * │  Y BUTTON: (unassigned — reserved for climber)                 │
   * │                                                                │
   * │  RIGHT BUMPER: Full auto-aim shooting + jostle — hold          │
   * │  LEFT BUMPER:  Auto-align to trench heading — hold             │
   * │                                                                │
   * │  D-PAD UP:    Spit fuel (same as left trigger)                 │
   * │  D-PAD DOWN:  Set odometry pose to alliance hub                │
   * │  D-PAD LEFT:  Zero robot heading                               │
   * │  D-PAD RIGHT: Tower shot (fixed RPM) — hold                   │
   * └─────────────────────────────────────────────────────────────────┘
   * </pre>
   */
  private void configureButtonBindings() {

    // ── Default Drive Command ─────────────────────────────────────────────────
    // Field-relative swerve drive with cubic input curves for fine control.
    // Right stick X controls rotation, scaled to 75% for smoother turning.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX() * 0.75));

    // ── RIGHT TRIGGER: Acquire game piece ─────────────────────────────────────
    // While held: extend intake and continuously run rollers to pull in game pieces.
    // On release: stop rollers but keep the intake extended (driver retracts with A button).
    controller
        .rightTrigger(0.5)
        .whileTrue(IntakeCommands.acquireGamePieceContinuous(intake))
        .onFalse(IntakeCommands.stopIntakeRollers(intake));

    // ── LEFT TRIGGER: Spit fuel ───────────────────────────────────────────────
    // While held: extend intake and reverse all mechanisms to eject game pieces.
    // On release: all mechanisms stop automatically (handled by GamePieceCommands).
    controller.leftTrigger(0.5).whileTrue(GamePieceCommands.spitFuel(intake, feeder, hopper));

    // ── A BUTTON: Retract intake ──────────────────────────────────────────────
    controller.a().onTrue(IntakeCommands.stowIntake(intake));

    // ── B BUTTON: Extend intake ───────────────────────────────────────────────
    controller.b().onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.EXTENDED));

    // ── RIGHT BUMPER: Full auto-aim shooting sequence ─────────────────────────
    // While held:
    //   1. Auto-aim: rotates the BACK of the robot (shooter) toward the active target
    //   2. Flywheel: spins up to the distance-based RPM
    //   3. Wait: until flywheel at speed AND robot facing target (with timeout)
    //   4. Feed: runs feeder and hopper to launch game pieces
    // On release: the command's finallyDo stops feeder, hopper, and sets flywheels to idle.
    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> RobotState.getInstance().getAngleToActiveTarget(),
                    true)
                .alongWith(ShooterCommands.shootToActiveTargetSequence(shooter, feeder, hopper)));

    // ── LEFT BUMPER: Auto-align to trench heading ─────────────────────────────
    // While held: snaps the robot to the nearest 0° or 180° heading for driving
    // straight through the trench. Driver keeps full translation control.
    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveSnapToNearestXHeading(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // ── D-PAD LEFT (270): Zero robot heading ──────────────────────────────────
    // Resets the gyro so the robot's current heading is treated as 180°.
    controller
        .pov(270)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg)),
                    drive)
                .ignoringDisable(true));

    // ── D-PAD DOWN (180): Set odometry pose to alliance hub ───────────────────
    // Teleports the robot's odometry to directly in front of the alliance hub.
    // Useful for testing shooting without driving across the field.
    controller
        .pov(180)
        .onTrue(
            Commands.runOnce(
                    () -> drive.setPose(RobotState.getInstance().getPoseInFrontOfAllianceHub()),
                    drive)
                .ignoringDisable(true));

    // ── D-PAD UP (0): Spit fuel ───────────────────────────────────────────────
    // Same as left trigger — extend intake and reverse all mechanisms.
    controller.pov(0).whileTrue(GamePieceCommands.spitFuel(intake, feeder, hopper));

    // ── D-PAD RIGHT (90): Tower shot ──────────────────────────────────────────
    // While held: runs the shooting sequence at a fixed tower RPM.
    // No auto-aim — the driver aims manually. Useful when near the tower.
    // On release: the command's finallyDo stops feeder, hopper, and returns flywheels to idle.
    controller.pov(90).whileTrue(ShooterCommands.shootFromTowerSequence(shooter, feeder, hopper));
  }

  // ==================== PUBLIC GETTERS ====================

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

  // ==================== AUTO PREVIEW & STARTING POSE CHECK ====================

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
