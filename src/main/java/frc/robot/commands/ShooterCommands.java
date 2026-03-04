package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
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
   * Creates a command to run the flywheel at a low idle speed. Keeps the flywheel spinning slowly
   * between shots to reduce spin-up time at the cost of more battery usage.
   *
   * <p>Idle speed is configured via {@link ShooterConstants.Software#IDLE_DUTY_CYCLE}.
   *
   * @param shooter The shooter subsystem
   * @return A command that sets the flywheel to idle speed once and finishes immediately
   */
  public static Command runFlywheelsAtIdle(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          shooter.setFlyWheelDutyCycle(ShooterConstants.Software.IDLE_DUTY_CYCLE);
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
            // Wait until BOTH the flywheel is at speed AND the robot is facing the target,
            // or give up waiting after 2 seconds (safety timeout)
            Commands.waitUntil(
                    () ->
                        shooter.areFlywheelsAtTargetSpeed()
                            && RobotState.getInstance().facingTarget.getAsBoolean())
                .withTimeout(2.0)
                .andThen(
                    FeederCommands.runFeederAtVoltage(feeder, Volts.of(9))
                        .andThen(HopperCommands.runHopperAtPercentOutput(hopper, 0.3))))
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
            // or give up waiting after 2 seconds (safety timeout)
            Commands.waitUntil(
                    () ->
                        shooter.areFlywheelsAtTargetSpeed()
                            && RobotState.getInstance().facingTarget.getAsBoolean())
                .withTimeout(2.0)
                .andThen(
                    FeederCommands.runFeederAtVoltage(feeder, Volts.of(9))
                        .andThen(HopperCommands.runHopperAtPercentOutput(hopper, 0.3))))
        .withName("ShootToActiveTargetSequence");
  }
}
