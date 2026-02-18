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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Drive subsystem controls the robot's swerve drivetrain.
 *
 * <p><b>Hardware Overview:</b>
 *
 * <ul>
 *   <li>Uses MK4i swerve modules with an inverted belly pan for electronics
 *   <li>Four independent swerve modules (Front-Left, Front-Right, Back-Left, Back-Right)
 *   <li>Each module has a drive motor and a turn motor controlled by REV Spark motor controllers
 *   <li>Gyroscope (Pigeon 2) for measuring robot rotation
 * </ul>
 *
 * <p><b>Software Features:</b>
 *
 * <ul>
 *   <li>Field-centric driving - robot moves relative to the field, not its own orientation
 *   <li>Odometry - tracks robot position on the field using wheel encoders and gyro
 *   <li>Vision integration - can accept camera measurements to improve position accuracy
 *   <li>PathPlanner integration - supports autonomous path following
 *   <li>Distance calculation to game elements (like the Hub for shooting)
 * </ul>
 *
 * <p>This subsystem uses the AdvantageKit IO layer pattern to separate hardware-specific code from
 * the main subsystem logic, making it easier to test and simulate.
 */
public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  /**
   * Creates a new Drive subsystem.
   *
   * <p>This constructor sets up the swerve drivetrain by initializing the gyroscope and all four
   * swerve modules. It also configures PathPlanner for autonomous driving and sets up the system
   * identification (SysId) routine for characterization.
   *
   * @param gyroIO The gyroscope hardware interface (NavX, Pigeon2, or simulation)
   * @param flModuleIO Front-left swerve module hardware interface
   * @param frModuleIO Front-right swerve module hardware interface
   * @param blModuleIO Back-left swerve module hardware interface
   * @param brModuleIO Back-right swerve module hardware interface
   */
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Drive/Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysId/State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  /**
   * Periodic method called every 20 milliseconds (50 times per second).
   *
   * <p>This method handles:
   *
   * <ul>
   *   <li>Reading sensor data from the gyro and all swerve modules
   *   <li>Logging all sensor data to AdvantageKit
   *   <li>Stopping the robot when disabled for safety
   *   <li>Updating odometry - calculating the robot's position on the field
   * </ul>
   *
   * <p><b>Odometry Update Process:</b>
   *
   * <ol>
   *   <li>Read wheel positions from each module (distance traveled and angle)
   *   <li>Calculate how far each wheel moved since last update (delta)
   *   <li>Read gyro angle to know robot's rotation
   *   <li>Combine wheel deltas and gyro angle to calculate robot's new position
   * </ol>
   *
   * <p>The odometry runs at a higher frequency than 50Hz by processing multiple samples per loop,
   * improving accuracy especially during fast movements.
   */
  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update to local pose estimator
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      // Also update the centralized RobotState
      // This allows other subsystems (like Shooter) to access robot state without
      // a direct reference to Drive
      RobotState.getInstance()
          .addOdometryObservation(
              sampleTimestamps[i], rawGyroRotation, modulePositions, getModuleStates());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * <p>This is the main method for controlling the robot's movement. It takes desired speeds for
   * the robot (forward, sideways, and rotation) and converts them into individual commands for each
   * swerve module.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Takes the desired ChassisSpeeds (overall robot velocity)
   *   <li>Discretizes the speeds to prevent skewing during fast rotation
   *   <li>Uses kinematics to calculate what each module needs to do
   *   <li>Desaturates wheel speeds if any exceed the maximum (scales all down proportionally)
   *   <li>Sends optimized setpoints to each module
   * </ol>
   *
   * <p><b>Note:</b> Module states are automatically optimized (shortest rotation path) by each
   * module's runSetpoint method.
   *
   * @param speeds Desired robot speeds (x, y velocities in m/s and rotation in rad/s). Positive x
   *     is forward, positive y is left, positive rotation is counterclockwise.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("Drive/Setpoints", setpointStates);
    Logger.recordOutput("Drive/ChassisSpeeds", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive in a straight line with the specified drive output.
   *
   * <p>This method is used for characterization (measuring how the drivetrain responds to different
   * voltages). During characterization, all modules point forward and receive the same voltage
   * output.
   *
   * <p><b>When is this used?</b> This is called by SysId routines to measure the drivetrain's
   * physical properties (like friction and motor characteristics) for better feedforward control.
   *
   * @param output The drive output voltage (-12 to +12 volts) to apply to all modules
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /**
   * Stops the drive by commanding zero velocity.
   *
   * <p>This sets all module velocities to zero but doesn't change their angles. The modules will
   * stay pointing in their current directions.
   */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   *
   * <p>The modules form an X pattern:
   *
   * <pre>
   *   \  /
   *    \/
   *    /\
   *   /  \
   * </pre>
   *
   * <p>This X configuration makes it very difficult for other robots to push you because the wheels
   * are braced against each other. This is useful for:
   *
   * <ul>
   *   <li>Defense - resisting being pushed by other robots
   *   <li>Stability - maintaining position on an incline or when hit
   *   <li>End of auto - preventing drift after autonomous ends
   * </ul>
   *
   * <p>The modules will return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Returns a command to run a quasistatic test in the specified direction.
   *
   * <p><b>What is a quasistatic test?</b> This slowly increases the voltage applied to the
   * drivetrain and measures how fast it moves.
   *
   * <p>This helps measure:
   *
   * <ul>
   *   <li>Static friction (kS) - voltage needed to overcome friction and start moving
   *   <li>Velocity constant (kV) - how voltage relates to velocity
   * </ul>
   *
   * <p><b>Usage:</b> Bind this to a button during characterization. Make sure the robot has plenty
   * of space to drive!
   *
   * @param direction The direction to run the test (forward or backward)
   * @return A command that runs the quasistatic characterization routine
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /**
   * Returns a command to run a dynamic test in the specified direction.
   *
   * <p><b>What is a dynamic test?</b> This applies a large voltage step to the drivetrain and
   * measures how quickly it accelerates. This happens fast - the robot will jerk forward!
   *
   * <p>This helps measure:
   *
   * <ul>
   *   <li>Acceleration constant (kA) - how voltage relates to acceleration
   * </ul>
   *
   * <p>Combined with quasistatic test data, this gives us a complete feedforward model for accurate
   * velocity control during autonomous.
   *
   * <p><b>Warning:</b> The robot will move quickly and suddenly. Make sure you have plenty of
   * space!
   *
   * @param direction The direction to run the test (forward or backward)
   * @return A command that runs the dynamic characterization routine
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the modules.
   *
   * <p>A SwerveModuleState contains:
   *
   * <ul>
   *   <li>Speed - how fast the wheel is spinning (meters per second)
   *   <li>Angle - which direction the wheel is pointing (Rotation2d)
   * </ul>
   *
   * <p>This method is automatically logged to AdvantageKit as "SwerveStates/Measured" so you can
   * visualize the actual module states in AdvantageScope.
   *
   * @return Array of 4 module states [Front-Left, Front-Right, Back-Left, Back-Right]
   */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the modules.
   *
   * <p>Similar to module states, but instead of velocity, this returns:
   *
   * <ul>
   *   <li>Distance - total distance the wheel has traveled (meters)
   *   <li>Angle - which direction the wheel is pointing (Rotation2d)
   * </ul>
   *
   * <p>This is used for odometry calculations to track the robot's position on the field.
   *
   * @return Array of 4 module positions [Front-Left, Front-Right, Back-Left, Back-Right]
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * Returns the measured chassis speeds of the robot.
   *
   * <p>This converts the individual module states back into overall robot movement:
   *
   * <ul>
   *   <li>vx - velocity forward/backward (meters per second)
   *   <li>vy - velocity left/right (meters per second)
   *   <li>omega - rotation speed (radians per second)
   * </ul>
   *
   * <p>This is automatically logged as "SwerveChassisSpeeds/Measured" so you can see the actual
   * robot velocity in AdvantageScope.
   *
   * @return The current chassis speeds based on measured module states
   */
  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the robot's velocity in field-relative coordinates.
   *
   * <p>This transforms the robot-relative chassis speeds into field coordinates, accounting for the
   * robot's heading. This is used by FuelSim to calculate projectile trajectories that account for
   * robot motion.
   *
   * <p><b>Coordinate Transformation:</b>
   *
   * <p>Robot-relative speeds are measured from the robot's perspective:
   *
   * <ul>
   *   <li>vx = forward/backward relative to robot
   *   <li>vy = left/right relative to robot
   * </ul>
   *
   * <p>Field-relative speeds are measured from the field's perspective:
   *
   * <ul>
   *   <li>vx = velocity toward the opposite end of the field
   *   <li>vy = velocity toward the left side of the field
   * </ul>
   *
   * <p>The transformation uses the robot's heading to rotate the velocity vector. For example, if
   * the robot is facing sideways (90°) and moving "forward" at 1 m/s, the field-relative velocity
   * would be 1 m/s to the left.
   *
   * @return Field-relative chassis speeds (vx, vy in field coordinates, omega unchanged)
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    Pose2d pose = getPose();
    ChassisSpeeds robotRelative = getChassisSpeeds();

    // Transform from robot-relative to field-relative
    // ChassisSpeeds.fromRobotRelativeSpeeds rotates the vx/vy by the robot's heading
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, pose.getRotation());
  }

  /**
   * Returns the position of each module in radians.
   *
   * <p>This is used during wheel radius characterization to determine the effective wheel radius.
   * The robot spins in place and we measure how far each wheel travels.
   *
   * @return Array of 4 wheel positions in radians [FL, FR, BL, BR]
   */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the average velocity of the modules in radians per second.
   *
   * <p>This is used during feedforward characterization to measure how the drivetrain responds to
   * different voltages. The velocity is measured in radians per second (wheel rotation speed), not
   * meters per second.
   *
   * @return Average module velocity in rad/sec
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /**
   * Returns the current odometry pose (position and rotation on the field).
   *
   * <p>The Pose2d contains:
   *
   * <ul>
   *   <li>X position - distance along the field length (meters)
   *   <li>Y position - distance along the field width (meters)
   *   <li>Rotation - which direction the robot is facing (Rotation2d)
   * </ul>
   *
   * <p>The origin (0, 0) is typically at the blue corner of the field, with positive X extending
   * toward the opposite corner and positive Y extending to the left.
   *
   * <p>This is automatically logged as "Odometry/Robot" and can be visualized on a field diagram in
   * AdvantageScope.
   *
   * @return The robot's current pose on the field
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the current odometry rotation (which direction the robot is facing).
   *
   * <p>This is just the rotation component of the pose. Useful when you only need to know the
   * robot's heading, not its position.
   *
   * @return The robot's current rotation on the field
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Resets the current odometry pose to a new position.
   *
   * <p>This tells the robot "you are now at this position on the field." Use this to:
   *
   * <ul>
   *   <li>Set the robot's starting position at the beginning of autonomous
   *   <li>Correct position estimates when you know the robot's true location
   *   <li>Reset to (0, 0) for testing
   * </ul>
   *
   * <p><b>Important:</b> Make sure the pose rotation matches the robot's actual heading, or
   * odometry will be inaccurate!
   *
   * @param pose The new pose to reset to (position and rotation on the field)
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a new timestamped vision measurement to improve odometry accuracy.
   *
   * <p>Vision systems (like Limelight or PhotonVision) can detect AprilTags on the field and
   * calculate the robot's position. This position can be added to the pose estimator to correct for
   * odometry drift.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Vision system detects AprilTag and calculates robot pose
   *   <li>This method adds that measurement with a timestamp (when it was captured)
   *   <li>Pose estimator combines vision data with wheel odometry
   *   <li>Less reliable measurements (far away, ambiguous) get lower weight via standard deviations
   * </ol>
   *
   * <p><b>Standard Deviations:</b> These tell the pose estimator how much to trust the measurement.
   * Smaller values = more trust, larger values = less trust. Typically: close/clear tags = small
   * stddev, far/unclear tags = large stddev.
   *
   * @param visionRobotPoseMeters The robot pose measured by the vision system
   * @param timestampSeconds When the measurement was captured (from Logger.getRealTimestamp())
   * @param visionMeasurementStdDevs Standard deviations for x, y, and rotation (3x1 matrix)
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Returns the maximum linear speed in meters per second.
   *
   * <p>This is the fastest the robot can drive in a straight line, determined by:
   *
   * <ul>
   *   <li>Motor free speed
   *   <li>Gear reduction
   *   <li>Wheel diameter
   * </ul>
   *
   * <p>Used by PathPlanner and other code that needs to know the robot's speed limits.
   *
   * @return Maximum speed in meters per second
   */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /**
   * Returns the maximum angular speed in radians per second.
   *
   * <p>This is how fast the robot can spin in place. It's calculated from the maximum linear speed
   * and the drive base radius (distance from center to wheels).
   *
   * <p>The formula: max angular speed = max linear speed / drive base radius <br>
   * Think of it like: the farther the wheels are from the center, the slower the robot spins for
   * the same wheel speed.
   *
   * <p>Used by PathPlanner and other code that needs to know rotation speed limits.
   *
   * @return Maximum rotation speed in radians per second
   */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}
