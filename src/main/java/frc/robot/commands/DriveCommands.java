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

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.robot.RobotState;
import frc.robot.RobotState.Target;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  // Note: Turn-to-angle PID constants have been moved to DriveConstants for easy tuning.
  // See DriveConstants.turnToAngleKP, turnToAngleKD, turnToAngleMaxVelocity,
  // turnToAngleMaxAcceleration
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  /**
   * Applies a cubic exponential curve to a joystick input value.
   *
   * <p>A cubic curve (x³) gives the driver very fine control at low stick deflections while still
   * allowing full speed at maximum deflection. This feels more natural than a linear response,
   * especially for precise movements.
   *
   * <p>Because x³ preserves the sign of x (e.g., (-0.5)³ = -0.125), no special sign handling is
   * needed — the direction of the input is always maintained.
   *
   * <p>Example values:
   *
   * <ul>
   *   <li>10% stick → 0.1³ = 0.001 (very slow)
   *   <li>50% stick → 0.5³ = 0.125 (moderate)
   *   <li>100% stick → 1.0³ = 1.0 (full speed)
   * </ul>
   *
   * @param value The raw joystick value, already deadbanded, in the range [-1, 1]
   * @return The curved output in the range [-1, 1]
   */
  private static double applyCubicCurve(double value) {
    // x^3 naturally preserves sign, so no Math.copySign() needed
    return value * value * value;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Apply cubic curve for more precise control at low speeds
    linearMagnitude = applyCubicCurve(linearMagnitude);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Apply cubic curve for more precise rotation control at low speeds.
          // applyCubicCurve() preserves the sign so the rotation direction is maintained.
          omega = applyCubicCurve(omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   *
   * <p>This overload defaults to aiming with the front of the robot.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, rotationSupplier, false);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   *
   * @param drive The drive subsystem
   * @param xSupplier Joystick X axis (left/right translation)
   * @param ySupplier Joystick Y axis (forward/back translation)
   * @param rotationSupplier The target field-relative angle to face
   * @param useBackOfRobot If true, rotates the target 180° so the back of the robot faces the
   *     target instead of the front. Useful for rear-mounted mechanisms (e.g. shooter).
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      boolean useBackOfRobot) {

    // Create PID controller using constants from DriveConstants so they're easy to tune
    // in one place without touching this file.
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.turnToAngleKP,
            0.0,
            DriveConstants.turnToAngleKD,
            new TrapezoidProfile.Constraints(
                DriveConstants.turnToAngleMaxVelocity, DriveConstants.turnToAngleMaxAcceleration));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Get the target angle, rotating 180° if we want the back to face the target
              Rotation2d targetAngle = rotationSupplier.get();
              if (useBackOfRobot) {
                targetAngle = targetAngle.plus(Rotation2d.kPi);
              }

              // Capture current heading for PID and logging
              Rotation2d currentAngle = drive.getRotation();

              // Calculate angular speed
              double omega =
                  angleController.calculate(currentAngle.getRadians(), targetAngle.getRadians());

              // Log PID setpoint and measurement so we can tune and debug in AdvantageScope
              // Rotation2d is a struct type — AdvantageKit logs it with built-in unit metadata
              Logger.recordOutput("Drive/AnglePID/Setpoint", targetAngle);
              Logger.recordOutput("Drive/AnglePID/Measurement", currentAngle);
              Logger.recordOutput("Drive/AnglePID/Error", targetAngle.minus(currentAngle));
              Logger.recordOutput("Drive/AnglePID/Omega", RadiansPerSecond.of(omega));

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Field relative drive command that snaps the robot to the nearest "straight" X-axis heading.
   *
   * <p>The two X-axis headings are 0° (facing the red alliance wall) and 180° (facing the blue
   * alliance wall). The command picks whichever heading is closest to the robot's current heading
   * so the robot doesn't spin around. For example:
   *
   * <ul>
   *   <li>If the robot is roughly facing the red wall (heading between -90° and 90°), it snaps to
   *       0°.
   *   <li>If the robot is roughly facing the blue wall (heading outside that range), it snaps to
   *       180°.
   * </ul>
   *
   * <p>This is useful for quickly straightening out to drive through the trench.
   *
   * @param drive The drive subsystem
   * @param xSupplier Joystick X axis (left/right translation)
   * @param ySupplier Joystick Y axis (forward/back translation)
   */
  public static Command joystickDriveSnapToNearestXHeading(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          // Get the robot's current heading in radians (-PI to PI)
          double currentRadians = drive.getRotation().getRadians();

          // If the absolute heading is <= 90° (PI/2), the robot is closer to 0° (facing red wall)
          // Otherwise, it's closer to 180° (facing blue wall)
          if (Math.abs(currentRadians) <= Math.PI / 2.0) {
            return Rotation2d.kZero; // Snap to 0°
          } else {
            return Rotation2d.kPi; // Snap to 180°
          }
        });
  }

  /**
   * Field relative drive command that rotates the robot so the intake (front) faces the alliance
   * wall.
   *
   * <p>Uses {@link AllianceFlipUtil} to determine which wall to face:
   *
   * <ul>
   *   <li><b>Blue alliance:</b> Intake faces 180° (toward the blue wall at X = 0)
   *   <li><b>Red alliance:</b> Intake faces 0° (toward the red wall at X = fieldLength)
   * </ul>
   *
   * <p>This is useful for lining up to drive through the trench with the intake leading.
   *
   * @param drive The drive subsystem
   * @param xSupplier Joystick X axis (left/right translation)
   * @param ySupplier Joystick Y axis (forward/back translation)
   */
  public static Command joystickDriveIntakeAtAllianceWall(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          // From blue perspective, intake (front) toward blue wall = 180°
          // AllianceFlipUtil.apply rotates by 180° for red alliance → 0° (toward red wall)
          return AllianceFlipUtil.apply(Rotation2d.kPi);
        });
  }

  // ==================== AUTO-DISTANCE DRIVE COMMAND ====================

  /**
   * Proportional gain for the radial (distance-from-hub) PID controller used by {@link
   * #joystickDriveAtAngleAndDistance}. Controls how aggressively the robot drives toward the target
   * distance.
   */
  private static final double DISTANCE_KP = 4.0;

  /**
   * Tolerance in meters for the distance controller. When the robot is within this distance of the
   * target radius, the radial drive effort drops to zero.
   */
  private static final double DISTANCE_TOLERANCE_METERS = 0.1; // ~4 inches

  /**
   * Field-relative drive command that auto-aims at the active target AND automatically drives to
   * the closest tuned shot distance from the hub.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Rotation: Uses a profiled PID controller to aim the back of the robot at the target (same
   *       as {@link #joystickDriveAtAngle} with useBackOfRobot=true)
   *   <li>Translation: The driver controls lateral (tangential) movement via the joystick.
   *       Additionally, this command adds a radial velocity component that drives the robot toward
   *       or away from the hub until it reaches the closest tuned shot distance (from {@link
   *       ShotCalculator#getClosestTunedHubDistanceMeters()}).
   * </ol>
   *
   * <p><b>When to use:</b> Bind this to a button for "auto-distance + auto-aim" shooting at the
   * hub. For feed targets, only the angle is auto-controlled — no distance adjustment.
   *
   * @param drive The drive subsystem
   * @param xSupplier Joystick X axis (forward/back translation)
   * @param ySupplier Joystick Y axis (left/right translation)
   * @param rotationSupplier Supplier for the target heading (e.g. angle to active target)
   * @return A command that auto-aims and auto-distances for hub shots
   */
  public static Command joystickDriveAtAngleAndDistance(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Rotation PID (same as joystickDriveAtAngle)
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.turnToAngleKP,
            0.0,
            DriveConstants.turnToAngleKD,
            new TrapezoidProfile.Constraints(
                DriveConstants.turnToAngleMaxVelocity, DriveConstants.turnToAngleMaxAcceleration));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // ── Driver linear velocity (joystick input) ──
              Translation2d driverLinearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // ── Rotation PID (aim back of robot at target) ──
              Rotation2d targetAngle = rotationSupplier.get().plus(Rotation2d.kPi);
              Rotation2d currentAngle = drive.getRotation();
              double omega =
                  angleController.calculate(currentAngle.getRadians(), targetAngle.getRadians());

              // ── Radial distance correction (hub only) ──
              double radialVx = 0.0;
              double radialVy = 0.0;

              Target activeTarget = RobotState.getInstance().getActiveTarget();
              if (activeTarget == Target.HUB) {
                Translation2d robotPos =
                    RobotState.getInstance().getEstimatedPose().getTranslation();
                Translation2d hubPos =
                    RobotState.getInstance().getAllianceHubTarget().toTranslation2d();
                double currentDistance = robotPos.getDistance(hubPos);
                OptionalDouble targetDistanceOpt =
                    ShotCalculator.getInstance().getClosestTunedHubDistanceMeters();

                // Only apply radial correction if a safe tuned distance exists.
                // If all tuned distances collide with the tower, skip distance
                // adjustment — the shooting command will block feeding instead.
                if (targetDistanceOpt.isPresent()) {
                  double targetDistance = targetDistanceOpt.getAsDouble();
                  double distanceError = currentDistance - targetDistance;

                  // Only apply correction if outside tolerance
                  if (Math.abs(distanceError) > DISTANCE_TOLERANCE_METERS) {
                    // P-controller: positive error = too far → drive toward hub (negative radial)
                    double radialSpeed = DISTANCE_KP * distanceError;

                    // Clamp to a reasonable fraction of max speed so the robot doesn't lurch
                    double maxRadialFraction = 0.7; // 60% of max speed
                    radialSpeed =
                        MathUtil.clamp(radialSpeed, -maxRadialFraction, maxRadialFraction);

                    // Convert radial direction to field-relative velocity components
                    // Direction from robot toward hub
                    Translation2d robotToHub = hubPos.minus(robotPos);
                    double dirAngle = Math.atan2(robotToHub.getY(), robotToHub.getX());

                    // Positive radialSpeed = drive toward hub, negative = drive away
                    radialVx = radialSpeed * Math.cos(dirAngle);
                    radialVy = radialSpeed * Math.sin(dirAngle);
                  }

                  Logger.recordOutput("Drive/AutoDistance/TargetDistance_m", targetDistance);
                  Logger.recordOutput("Drive/AutoDistance/CurrentDistance_m", currentDistance);
                  Logger.recordOutput("Drive/AutoDistance/DistanceError_m", distanceError);
                }

                Logger.recordOutput("Drive/AutoDistance/Blocked", targetDistanceOpt.isEmpty());
              }

              Logger.recordOutput("Drive/AnglePID/Setpoint", targetAngle);
              Logger.recordOutput("Drive/AnglePID/Measurement", currentAngle);
              Logger.recordOutput("Drive/AnglePID/Error", targetAngle.minus(currentAngle));
              Logger.recordOutput("Drive/AnglePID/Omega", RadiansPerSecond.of(omega));

              // ── Combine driver input + radial correction ──
              double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      driverLinearVelocity.getX() * maxSpeed + radialVx * maxSpeed,
                      driverLinearVelocity.getY() * maxSpeed + radialVy * maxSpeed,
                      omega);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .withName("JoystickDriveAtAngleAndDistance");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
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

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

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
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
