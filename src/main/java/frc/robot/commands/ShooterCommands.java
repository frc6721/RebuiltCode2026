package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

/**
 * Factory class for creating shooter-related commands.
 *
 * <p>Provides static factory methods for manual speed control, distance-based shooting, dynamic
 * hub-tracking, and flywheel idle management.
 */
public class ShooterCommands {

  // Feedforward characterization tuning constants
  private static final double FF_START_DELAY = 2.0; // seconds to wait for idle before ramp
  private static final double FF_RAMP_RATE = 0.1; // Volts per second

  /**
   * Creates a command to set the flywheel to a specific target speed. Sets speed once and finishes
   * immediately - the motor controller's PID maintains the speed afterward.
   *
   * <p>Use {@link #waitForFlywheelsToReachSpeed(Shooter)} before feeding to ensure accuracy.
   *
   * @param shooter The shooter subsystem
   * @param speed The target flywheel speed (use RPM.of(), RadiansPerSecond.of(), etc.)
   * @return A command that sets the flywheel target speed once and finishes immediately
   */
  public static Command setFlywheelTargetSpeed(Shooter shooter, AngularVelocity speed) {
    return Commands.runOnce(
        () -> {
          shooter.setFlywheelSpeed(speed);
        },
        shooter);
  }

  public static Command increaseFlywheelRPMOffset(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          shooter.incFlywheelRPMOffset(25.0);
        },
        shooter);
  }

  public static Command decreaseFlywheelRPMOffset(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          shooter.incFlywheelRPMOffset(-25.0);
        },
        shooter);
  }

  public static Command runShooterAndFeederAtVoltage(
      Shooter shooter, Feeder feeder, double shooterVoltage, double feederVoltage) {
    // Run shooter continuously in parallel with a delayed feeder start.
    // The shooter spins up immediately; after 1.5s the feeder begins feeding.
    // Both run until the command is interrupted (button released).
    return Commands.runOnce(
            () -> {
              shooter.runCharacterization(Volts.of(shooterVoltage));
            },
            shooter)
        .alongWith(
            new WaitCommand(1.5)
                .andThen(
                    Commands.runOnce(
                        () -> {
                          feeder.runFeederAtVoltage(Volts.of(feederVoltage));
                        },
                        feeder)))
        .withName("RunShooterAndFeederAtVoltage");
  }

  /**
   * Creates a command to stop the flywheel. Runs once and finishes immediately - the flywheel will
   * coast to a stop due to momentum.
   *
   * <p>Consider using {@link #runFlywheelsAtIdle(Shooter)} instead for faster spin-up on next shot.
   *
   * @param shooter The shooter subsystem
   * @return A command that stops the flywheel once and finishes immediately
   */
  public static Command stopFlywheels(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          shooter.stopFlywheels();
        },
        shooter);
  }

  /**
   * Creates a command to set the flywheel to idle (0 RPM) using velocity control mode. Keeps the
   * shooter in closed-loop velocity mode so {@link Shooter#areFlywheelsAtTargetSpeed()} returns
   * accurate results and the target speed is properly reset.
   *
   * @param shooter The shooter subsystem
   * @return A command that sets the flywheel to 0 RPM once and finishes immediately
   */
  public static Command runFlywheelsAtIdle(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          shooter.setFlywheelSpeed(RPM.of(0));
        },
        shooter);
  }

  /**
   * Creates a command to set flywheel speed based on distance to the target. Uses the interpolating
   * lookup table in ShooterConstants to calculate RPM. Sets speed once and finishes immediately.
   *
   * <p><b>Note:</b> Speed is calculated at the moment the command runs. For continuously updating
   * speed as the robot moves, use {@link #shootToHub(Shooter)} instead.
   *
   * @param shooter The shooter subsystem
   * @param distance Distance to the target (from odometry or vision)
   * @return A command that calculates and sets the correct flywheel speed once, then finishes
   */
  public static Command setFlywheelSpeedForDistance(Shooter shooter, Distance distance) {
    return Commands.runOnce(
        () -> {
          AngularVelocity targetSpeed = shooter.getSpeedForDistance(distance);
          shooter.setFlywheelSpeed(targetSpeed);
        },
        shooter);
  }

  /**
   * Creates a command that waits until the flywheel reaches its target speed (within tolerance).
   *
   * <p>Always use this before feeding game pieces - feeding before the flywheel is at speed results
   * in weak, inconsistent shots. Consider adding a timeout: {@code .withTimeout(2.0)}.
   *
   * @param shooter The shooter subsystem
   * @return A command that finishes when the flywheel reaches target speed
   */
  public static Command waitForFlywheelsToReachSpeed(Shooter shooter) {
    return Commands.waitUntil(() -> shooter.areFlywheelsAtTargetSpeed());
  }

  /**
   * Simple feedforward characterization for the shooter flywheel.
   *
   * <p>Ramps voltage up linearly while sampling the resulting angular velocity. When the command is
   * canceled, performs a least-squares fit to compute kS and kV (volts and volts per RPM
   * respectively) and prints the results.
   *
   * <p>Use this in voltage-control mode only. Example: hold the X button to run the
   * characterization, release to finish and print results.
   *
   * <p><b>Units:</b> kS is in Volts, kV is in Volts per RPM
   */
  public static Command feedforwardCharacterization(Shooter shooter) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Ensure idle for a moment so things settle
        Commands.run(() -> shooter.runCharacterization(Volts.of(0.0)), shooter)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Ramp voltage and sample
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  shooter.runCharacterization(Volts.of(voltage));
                  // Sample velocity in rad/s, convert to RPM using WPILib units
                  double velocityRadPerSec = shooter.getFFCharacterizationVelocity();
                  double velocityRPM = RadiansPerSecond.of(velocityRadPerSec).in(RPM);
                  velocitySamples.add(velocityRPM);
                  voltageSamples.add(voltage);
                },
                shooter)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Shooter FF Characterization Results **********");
                  System.out.println("\tkS (Volts): " + formatter.format(kS));
                  System.out.println("\tkV (Volts per RPM): " + formatter.format(kV));

                  shooter.stopFlywheels();
                }));
  }

  // ==================== DYNAMIC SHOOTING COMMANDS ====================

  /**
   * Creates a command to continuously update flywheel speed based on distance to the alliance hub.
   * Updates every 20ms as the robot moves, with auto alliance-flip.
   *
   * <p>Runs until interrupted - typically used with {@code whileTrue()} button binding.
   *
   * @param shooter The shooter subsystem
   * @return A command that continuously updates flywheel speed for the alliance hub
   */
  public static Command shootToHub(Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.updateSpeedForHub();
            },
            shooter)
        .withName("ShootToHub");
  }

  public static Command shootToHub(Shooter shooter, double rpm) {
    return Commands.run(
            () -> {
              shooter.setFlywheelSpeed(RPM.of(rpm));
            },
            shooter)
        .withName("ShootToHub");
  }

  /**
   * Creates a command to continuously update flywheel speed based on distance to a specific target.
   *
   * <p>Similar to shootToHub(), but allows shooting at any point on the field. Useful for shooting
   * at specific targets or testing different positions.
   *
   * @param shooter The shooter subsystem
   * @param target The target point to shoot at (Translation3d in field coordinates)
   * @return A command that continuously updates flywheel speed for the target
   */
  public static Command shootToPoint(
      Shooter shooter, edu.wpi.first.math.geometry.Translation3d target) {
    return Commands.run(
            () -> {
              shooter.updateSpeedForTarget(target);
            },
            shooter)
        .withName("ShootToPoint");
  }

  /**
   * Creates a complete shooting sequence: continuously tracks hub distance, waits for flywheel to
   * reach speed AND robot to face the target (2s timeout), then feeds the game piece.
   *
   * <p>The robot will only feed when BOTH conditions are met:
   *
   * <ul>
   *   <li>Flywheel is at target speed (within PID tolerance)
   *   <li>Robot is facing the active target (within {@link
   *       frc.robot.RobotState#SHOOT_TOLERANCE_DEGREES} degrees)
   * </ul>
   *
   * <p>If neither condition is met within 2 seconds, feeding begins anyway as a safety timeout.
   *
   * @param shooter The shooter subsystem
   * @param feeder The feeder subsystem
   * @param hopper The hopper subsystem
   * @return A complete shooting sequence command
   */
  public static Command shootToHubSequence(
      Shooter shooter, frc.robot.subsystems.feeder.Feeder feeder, Hopper hopper) {
    return shootToHub(shooter)
        .alongWith(
            // Wait until BOTH the flywheel is at speed AND the robot's back is facing the hub,
            // or give up waiting after the configured timeout (safety fallback)
            Commands.waitUntil(
                    () ->
                        shooter.areFlywheelsAtTargetSpeed()
                            && RobotState.getInstance().isBackFacingAllianceHub())
                .withTimeout(ShooterConstants.Software.SHOOT_SEQUENCE_TIMEOUT_SECONDS)
                .andThen(
                    // startEnd() runs the motor on start and stops it when interrupted/finished,
                    // so the feeder and hopper always stop cleanly when the sequence ends.
                    Commands.startEnd(
                        () -> {
                          feeder.setFeederVelocity(FeederConstants.Speeds.SHOOT_FEED_RPM);
                          hopper.setHopperSpeed(HopperConstants.Speeds.FEED_SPEED);
                        },
                        () -> {
                          feeder.stop();
                          hopper.stop();
                        },
                        feeder,
                        hopper)))
        .withName("ShootToHubSequence");
  }

  public static Command shootToHubSequence(
      Shooter shooter, frc.robot.subsystems.feeder.Feeder feeder, Hopper hopper, double rpm) {
    return shootToHub(shooter, rpm)
        .alongWith(
            // Wait until BOTH the flywheel is at speed AND the robot's back is facing the hub,
            // or give up waiting after the configured timeout (safety fallback)
            Commands.waitUntil(
                    () ->
                        shooter.areFlywheelsAtTargetSpeed()
                            && RobotState.getInstance().isBackFacingAllianceHub())
                .withTimeout(ShooterConstants.Software.SHOOT_SEQUENCE_TIMEOUT_SECONDS)
                .andThen(
                    // startEnd() runs the motor on start and stops it when interrupted/finished,
                    // so the feeder and hopper always stop cleanly when the sequence ends.
                    Commands.startEnd(
                        () -> {
                          feeder.setFeederVelocity(FeederConstants.Speeds.SHOOT_FEED_RPM);
                          hopper.setHopperSpeed(HopperConstants.Speeds.FEED_SPEED);
                        },
                        () -> {
                          feeder.stop();
                          hopper.stop();
                        },
                        feeder,
                        hopper)))
        .withName("ShootToHubSequence");
  }

  // ==================== ACTIVE TARGET COMMANDS ====================

  /**
   * Creates a command to continuously update flywheel speed for the active target.
   *
   * <p>The active target is determined by the robot's field position via {@link
   * RobotState#getActiveTarget()}:
   *
   * <ul>
   *   <li>Alliance zone or trench → HUB (direct shot with hub speed map)
   *   <li>Neutral zone or opponent zone → FEED_LEFT or FEED_RIGHT (lob shot with feed speed map)
   * </ul>
   *
   * <p>Updates every 20ms as the robot moves. The target and speed map can change mid-command if
   * the robot crosses a zone boundary.
   *
   * <p>Runs until interrupted — typically used with {@code whileTrue()} button binding.
   *
   * @param shooter The shooter subsystem
   * @return A command that continuously updates flywheel speed for the active target
   */
  public static Command shootToActiveTarget(Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.updateSpeedForActiveTarget();
            },
            shooter)
        .withName("ShootToActiveTarget");
  }

  /**
   * Creates a complete shooting sequence that targets the active target based on field position.
   *
   * <p>This is the primary "shoot" command for teleop. It:
   *
   * <ol>
   *   <li>Continuously adjusts flywheel speed for the active target (hub or feed)
   *   <li>Waits for the flywheel to reach speed AND the robot to face the target (2s timeout)
   *   <li>Runs the feeder and hopper to launch the game piece
   * </ol>
   *
   * <p>The robot will only feed when BOTH conditions are met:
   *
   * <ul>
   *   <li>Flywheel is at target speed (within PID tolerance)
   *   <li>Robot is facing the active target (within {@link
   *       frc.robot.RobotState#SHOOT_TOLERANCE_DEGREES} degrees)
   * </ul>
   *
   * <p>If neither condition is met within 2 seconds, feeding begins anyway as a safety timeout.
   *
   * <p>Pair this with {@link frc.robot.commands.DriveCommands#joystickDriveAtAngle} using {@link
   * RobotState#getAngleToActiveTarget()} for full auto-aim + auto-shoot.
   *
   * @param shooter The shooter subsystem
   * @param feeder The feeder subsystem
   * @param hopper The hopper subsystem
   * @return A complete shooting sequence command targeting the active target
   */
  public static Command shootToActiveTargetSequence(Shooter shooter, Feeder feeder, Hopper hopper) {
    return shootToActiveTarget(shooter)
        .alongWith(
            // Wait until BOTH the flywheel is at speed AND the robot is facing the target,
            // or give up waiting after the configured timeout (safety fallback)
            Commands.waitUntil(
                    () ->
                        shooter.areFlywheelsAtTargetSpeed()
                            && RobotState.getInstance().facingTarget.getAsBoolean())
                .withTimeout(ShooterConstants.Software.SHOOT_SEQUENCE_TIMEOUT_SECONDS)
                .andThen(
                    // startEnd() runs the motor on start and stops it when interrupted/finished,
                    // so the feeder and hopper always stop cleanly when the sequence ends.
                    Commands.startEnd(
                        () -> {
                          feeder.setFeederVelocity(FeederConstants.Speeds.SHOOT_FEED_RPM);
                          hopper.setHopperSpeed(HopperConstants.Speeds.FEED_SPEED);
                        },
                        () -> {
                          feeder.stop();
                          hopper.stop();
                        },
                        feeder,
                        hopper)))
        // When the command ends (button released or interrupted), clean up all mechanisms.
        // This replaces the separate onFalse handler in RobotContainer.
        .finallyDo(
            () -> {
              feeder.stop();
              hopper.stop();
              shooter.setFlywheelSpeed(
                  RPM.of(0).minus(RPM.of(shooter.getFlyWheelRPMOffset()))); // undo flywheel offset
            })
        .withName("ShootToActiveTargetSequence");
  }

  public static Command shootToActiveTargetSequence(
      Shooter shooter, Feeder feeder, Hopper hopper, double RPM) {
    return shootToHub(shooter, RPM)
        .alongWith(
            // Wait until BOTH the flywheel is at speed AND the robot is facing the target,
            // or give up waiting after the configured timeout (safety fallback)
            Commands.waitUntil(
                    () ->
                        shooter.areFlywheelsAtTargetSpeed()
                            && RobotState.getInstance().facingTarget.getAsBoolean())
                .withTimeout(ShooterConstants.Software.SHOOT_SEQUENCE_TIMEOUT_SECONDS)
                .andThen(
                    // startEnd() runs the motor on start and stops it when interrupted/finished,
                    // so the feeder and hopper always stop cleanly when the sequence ends.
                    Commands.startEnd(
                        () -> {
                          feeder.setFeederVelocity(FeederConstants.Speeds.SHOOT_FEED_RPM);
                          hopper.setHopperSpeed(HopperConstants.Speeds.FEED_SPEED);
                        },
                        () -> {
                          feeder.stop();
                          hopper.stop();
                        },
                        feeder,
                        hopper)))
        // When the command ends (button released or interrupted), clean up all mechanisms.
        // This replaces the separate onFalse handler in RobotContainer.
        .finallyDo(
            () -> {
              feeder.stop();
              hopper.stop();
              shooter.setFlywheelSpeed(RotationsPerSecond.of(0));
            })
        .withName("ShootToActiveTargetSequence");
  }

  // ==================== TOWER SHOT COMMANDS ====================

  /**
   * Creates a command to set the flywheel to the fixed tower shot RPM. Uses the constant {@link
   * ShooterConstants.Software#TOWER_SHOT_RPM} instead of calculating speed from distance.
   *
   * <p>Use this when the robot is at the tower (close range) and distance-based aiming is not
   * needed.
   *
   * @param shooter The shooter subsystem
   * @return A command that continuously maintains the tower shot RPM
   */
  public static Command shootFromTower(Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.setFlywheelSpeed(ShooterConstants.Software.TOWER_SHOT_RPM);
            },
            shooter)
        .withName("ShootFromTower");
  }

  /**
   * Creates a complete tower shot shooting sequence: spins flywheel to tower RPM, waits for speed,
   * then feeds game pieces.
   *
   * <p>Unlike {@link #shootToHubSequence}, this uses a fixed RPM from {@link
   * ShooterConstants.Software#TOWER_SHOT_RPM} rather than distance-based speed calculation. No
   * auto-aim is included — pair with a drive command for aiming if needed.
   *
   * @param shooter The shooter subsystem
   * @param feeder The feeder subsystem
   * @param hopper The hopper subsystem
   * @return A complete tower shot sequence command
   */
  public static Command shootFromTowerSequence(Shooter shooter, Feeder feeder, Hopper hopper) {
    return shootFromTower(shooter)
        .alongWith(
            Commands.waitUntil(() -> shooter.areFlywheelsAtTargetSpeed())
                .withTimeout(ShooterConstants.Software.SHOOT_SEQUENCE_TIMEOUT_SECONDS)
                .andThen(
                    Commands.startEnd(
                        () -> {
                          feeder.setFeederVelocity(FeederConstants.Speeds.SHOOT_FEED_RPM);
                          hopper.setHopperSpeed(HopperConstants.Speeds.FEED_SPEED);
                        },
                        () -> {
                          feeder.stop();
                          hopper.stop();
                        },
                        feeder,
                        hopper)))
        // When the command ends (button released or interrupted), clean up all mechanisms.
        // This replaces the separate onFalse handler in RobotContainer.
        .finallyDo(
            () -> {
              feeder.stop();
              hopper.stop();
              shooter.setFlywheelSpeed(RPM.of(0));
            })
        .withName("ShootFromTowerSequence");
  }

  // ==================== MIN DISTANCE RESTRICTION COMMANDS ====================

  /**
   * Enables the minimum shooting distance restriction on the shooter.
   *
   * <p>When enabled, the shooter will not feed if the robot is closer to the hub than {@link
   * ShooterConstants.Software#HUB_MIN_SHOOTING_DISTANCE}. This only affects hub shots — feed shots
   * (left/right) are never restricted.
   *
   * @param shooter The shooter subsystem
   * @return A command that enables the restriction and finishes immediately
   */
  public static Command enableMinDistance(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setMinDistanceEnabled(true))
        .withName("EnableMinDistance");
  }

  /**
   * Disables the minimum shooting distance restriction on the shooter.
   *
   * <p>Useful for close-range "dump" shots or emergency overrides during competition.
   *
   * @param shooter The shooter subsystem
   * @return A command that disables the restriction and finishes immediately
   */
  public static Command disableMinDistance(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setMinDistanceEnabled(false))
        .withName("DisableMinDistance");
  }

  /**
   * Toggles the minimum shooting distance restriction on or off.
   *
   * @param shooter The shooter subsystem
   * @return A command that toggles the restriction and finishes immediately
   */
  public static Command toggleMinDistance(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setMinDistanceEnabled(!shooter.isMinDistanceEnabled()))
        .withName("ToggleMinDistance");
  }

  /**
   * Creates a command that rumbles the driver controller while shooting is blocked.
   *
   * <p>Runs continuously — rumbles while the shooter reports shooting is blocked (too close to
   * hub), and stops the rumble when shooting is no longer blocked or the command ends.
   *
   * <p>Should be run in parallel with a shooting command to provide haptic feedback.
   *
   * @param shooter The shooter subsystem
   * @param controller The driver's Xbox controller to rumble
   * @return A command that rumbles when shooting is blocked, stops rumble on end
   */
  public static Command rumbleWhenBlocked(Shooter shooter, CommandXboxController controller) {
    return Commands.run(
            () -> {
              if (shooter.isShootingBlocked()) {
                controller.getHID().setRumble(RumbleType.kRightRumble, 0.8);
              } else {
                controller.getHID().setRumble(RumbleType.kRightRumble, 0.0);
              }
            })
        .finallyDo(() -> controller.getHID().setRumble(RumbleType.kRightRumble, 0.0))
        .withName("RumbleWhenBlocked");
  }

  // ==================== AUTO-DISTANCE SHOOTING COMMANDS ====================

  /**
   * Creates a complete shooting sequence with automatic distance adjustment for the active target.
   *
   * <p>This is the "enhanced" version of {@link #shootToActiveTargetSequence}. It works the same
   * way, but with the following additions:
   *
   * <ul>
   *   <li>If the active target is the HUB, the robot will automatically drive to the nearest
   *       "tuned" shot distance from the hub (from {@link
   *       ShooterConstants.DistanceMap#TUNED_HUB_SHOT_DISTANCES_METERS})
   *   <li>If the active target is a feed, the robot will NOT auto-adjust distance (same behavior as
   *       the standard sequence)
   *   <li>The minimum distance restriction is respected — if blocked, the feeder will not run
   * </ul>
   *
   * <p>The old commands ({@link #shootToActiveTargetSequence}) are preserved for easy revert.
   *
   * @param shooter The shooter subsystem
   * @param feeder The feeder subsystem
   * @param hopper The hopper subsystem
   * @return A complete shooting sequence with auto-distance for hub shots
   */
  public static Command shootToActiveTargetWithAutoDistanceSequence(
      Shooter shooter, Feeder feeder, Hopper hopper) {
    return shootToActiveTarget(shooter)
        .alongWith(
            // Wait until the flywheel is at speed, robot is facing the target, AND
            // shooting is not blocked by the min distance restriction
            Commands.waitUntil(
                    () ->
                        shooter.areFlywheelsAtTargetSpeed()
                            && RobotState.getInstance().facingTarget.getAsBoolean()
                            && !shooter.isShootingBlocked())
                .withTimeout(ShooterConstants.Software.SHOOT_SEQUENCE_TIMEOUT_SECONDS)
                .andThen(
                    // Only feed if shooting is not blocked
                    Commands.either(
                        // ALLOWED: run feeder and hopper normally
                        Commands.startEnd(
                            () -> {
                              feeder.setFeederVelocity(FeederConstants.Speeds.SHOOT_FEED_RPM);
                              hopper.setHopperSpeed(HopperConstants.Speeds.FEED_SPEED);
                            },
                            () -> {
                              feeder.stop();
                              hopper.stop();
                            },
                            feeder,
                            hopper),
                        // BLOCKED: do nothing (just idle — the rumble command handles feedback)
                        Commands.none(),
                        // Condition: shoot only if NOT blocked
                        () -> !shooter.isShootingBlocked())))
        .finallyDo(
            () -> {
              feeder.stop();
              hopper.stop();
              shooter.setFlywheelSpeed(RPM.of(0));
            })
        .withName("ShootToActiveTargetWithAutoDistanceSequence");
  }
}
