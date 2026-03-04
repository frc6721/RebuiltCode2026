package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTrigger;
import frc.robot.util.PointInPolygon;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized robot state container that tracks the robot's position and velocity on the field.
 *
 * <p>This is a singleton class that provides a single source of truth for:
 *
 * <ul>
 *   <li>Robot pose (position and rotation on the field)
 *   <li>Robot velocity (field-relative and robot-relative)
 *   <li>Distance calculations to field elements (hubs)
 * </ul>
 *
 * <p><b>Why use a centralized RobotState?</b>
 *
 * <ul>
 *   <li>Decouples subsystems - Shooter doesn't need a reference to Drive
 *   <li>Single source of truth - All subsystems see the same robot state
 *   <li>Easier testing - Can mock robot state for unit tests
 *   <li>Cleaner architecture - Subsystems don't need pose/speed suppliers
 * </ul>
 *
 * <p><b>Usage:</b>
 *
 * <pre>
 * // Get the singleton instance
 * RobotState state = RobotState.getInstance();
 *
 * // Get current pose
 * Pose2d pose = state.getEstimatedPose();
 *
 * // Get distance to alliance hub (auto-flips for red alliance)
 * Distance distance = state.getDistanceToAllianceHub();
 * </pre>
 *
 * <p><b>Odometry Updates:</b> The Drive subsystem calls addOdometryObservation() during its
 * periodic loop to update the pose estimator with wheel encoder and gyro data.
 */
public class RobotState {

  // ==================== SINGLETON PATTERN ====================

  private static RobotState instance;

  /**
   * Returns the singleton instance of RobotState.
   *
   * <p>Creates the instance on first call (lazy initialization).
   *
   * @return The RobotState singleton
   */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // ==================== POSE ESTIMATION ====================

  /** Kinematics for converting between chassis speeds and module states */
  private final SwerveDriveKinematics kinematics;

  /** Pose estimator that fuses odometry and vision measurements */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Last known gyro rotation (used for odometry updates) */
  private Rotation2d rawGyroRotation = new Rotation2d();

  /** Last known module positions (used for delta calculations) */
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /** Current module states for velocity calculation */
  private SwerveModuleState[] currentModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  // ==================== CONSTRUCTOR ====================

  /** Private constructor - use getInstance() instead. */
  private RobotState() {
    // Initialize kinematics with module positions from DriveConstants
    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    // Initialize pose estimator at origin
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  }

  // ==================== POSE GETTERS ====================

  /**
   * Returns the current estimated robot pose on the field.
   *
   * <p>The pose includes:
   *
   * <ul>
   *   <li>X position - distance along the field length (meters)
   *   <li>Y position - distance along the field width (meters)
   *   <li>Rotation - which direction the robot is facing
   * </ul>
   *
   * <p>The origin (0, 0) is at the blue alliance corner, with positive X toward the red alliance
   * and positive Y to the left when looking from the blue alliance.
   *
   * @return The robot's current estimated pose
   */
  public Pose2d getEstimatedPose() {
    Pose2d pose = poseEstimator.getEstimatedPosition();
    Logger.recordOutput("RobotState/EstimatedPose", pose);
    return pose;
  }

  /**
   * Returns the robot's current rotation (heading) on the field.
   *
   * @return The robot's current rotation
   */
  public Rotation2d getRotation() {
    return getEstimatedPose().getRotation();
  }

  // ==================== VELOCITY GETTERS ====================

  /**
   * Returns the robot's velocity in field-relative coordinates.
   *
   * <p>Field-relative means the velocity is measured from the field's perspective:
   *
   * <ul>
   *   <li>vx = velocity toward the red alliance (positive X direction)
   *   <li>vy = velocity toward the left side of the field (positive Y direction)
   *   <li>omega = rotational velocity (counterclockwise positive)
   * </ul>
   *
   * <p>This is used by the shooter to compensate for robot motion when calculating shot
   * trajectories.
   *
   * @return Field-relative chassis speeds
   */
  public ChassisSpeeds getFieldRelativeVelocity() {
    ChassisSpeeds robotRelative = getRobotRelativeVelocity();
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getRotation());
    Logger.recordOutput("RobotState/FieldRelativeVelocity", fieldRelative);
    return fieldRelative;
  }

  /**
   * Returns the robot's velocity in robot-relative coordinates.
   *
   * <p>Robot-relative means the velocity is measured from the robot's perspective:
   *
   * <ul>
   *   <li>vx = forward velocity (positive = driving forward)
   *   <li>vy = left velocity (positive = strafing left)
   *   <li>omega = rotational velocity (counterclockwise positive)
   * </ul>
   *
   * @return Robot-relative chassis speeds
   */
  public ChassisSpeeds getRobotRelativeVelocity() {
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(currentModuleStates);
    Logger.recordOutput("RobotState/RobotRelativeVelocity", speeds);
    return speeds;
  }

  // ==================== DISTANCE CALCULATIONS ====================

  /**
   * Returns the 2D distance from the robot to the alliance hub.
   *
   * <p>This automatically flips based on alliance color:
   *
   * <ul>
   *   <li>Blue alliance → returns distance to blue hub
   *   <li>Red alliance → returns distance to red hub
   * </ul>
   *
   * <p>The distance is calculated to the top center point of the hub (the scoring target).
   *
   * @return Distance to the alliance hub as a Distance measure
   */
  public Distance getDistanceToAllianceHub() {
    Translation3d hubTarget = getAllianceHubTarget();
    Distance distance = getDistanceToPoint(hubTarget.toTranslation2d());
    Logger.recordOutput("RobotState/DistanceToAllianceHub_m", distance.in(Meters));
    return distance;
  }

  /**
   * Returns the 2D distance from the robot to the opposing alliance hub.
   *
   * <p>This automatically flips based on alliance color:
   *
   * <ul>
   *   <li>Blue alliance → returns distance to red hub
   *   <li>Red alliance → returns distance to blue hub
   * </ul>
   *
   * @return Distance to the opposing hub as a Distance measure
   */
  public Distance getDistanceToOpposingHub() {
    Translation3d hubTarget = getOpposingHubTarget();
    Distance distance = getDistanceToPoint(hubTarget.toTranslation2d());
    Logger.recordOutput("RobotState/DistanceToOpposingHub_m", distance.in(Meters));
    return distance;
  }

  /**
   * Returns the 2D distance from the robot to an arbitrary point on the field.
   *
   * @param point The target point (2D field coordinates)
   * @return Distance to the point
   */
  public Distance getDistanceToPoint(Translation2d point) {
    Translation2d robotPosition = getEstimatedPose().getTranslation();
    double distanceMeters = robotPosition.getDistance(point);
    return Meters.of(distanceMeters);
  }

  /**
   * Returns the 3D position of the alliance hub target (top center).
   *
   * <p>Auto-flips based on alliance color.
   *
   * @return The alliance hub target position
   */
  public Translation3d getAllianceHubTarget() {
    // FieldConstants.Hub.topCenterPoint is defined from blue alliance perspective
    // AllianceFlipUtil.apply() flips it if we're on red alliance
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
  }

  /**
   * Returns the 3D position of the opposing alliance hub target.
   *
   * <p>Auto-flips based on alliance color.
   *
   * @return The opposing hub target position
   */
  public Translation3d getOpposingHubTarget() {
    // oppTopCenterPoint is the opposing hub from blue perspective
    // Flip it if we're on red alliance
    return AllianceFlipUtil.apply(FieldConstants.Hub.oppTopCenterPoint);
  }

  /**
   * Returns the Pose2d directly in front of the alliance hub, 6 feet toward the alliance wall.
   *
   * <p>The position is calculated by taking the hub's center X and subtracting 6 feet (toward the
   * blue alliance wall). The robot is oriented to face the hub (pointing in the +X direction toward
   * it). {@link AllianceFlipUtil} automatically mirrors the pose for red alliance.
   *
   * <p><b>Usage:</b>
   *
   * <pre>
   * drive.setPose(RobotState.getInstance().getPoseInFrontOfAllianceHub());
   * </pre>
   *
   * @return A Pose2d 6 feet in front of the alliance hub, facing the hub
   */
  public Pose2d getPoseInFrontOfAllianceHub() {
    // Distance to stand in front of the hub (3.5 feet converted to meters)
    Distance offsetDistance = Feet.of(3.5);

    // Hub center (2D), defined from blue alliance perspective
    Translation2d hubCenter = FieldConstants.Hub.topCenterPoint.toTranslation2d();

    // Step 6 feet toward the blue alliance wall (negative X direction from the hub center)
    Translation2d positionBlue =
        new Translation2d(hubCenter.getX() - offsetDistance.in(Meters), hubCenter.getY());

    // Build the pose facing the hub (+X direction = toward hub) before flipping
    Pose2d poseBlue = new Pose2d(positionBlue, Rotation2d.kZero);

    // AllianceFlipUtil mirrors both position and rotation for red alliance
    return AllianceFlipUtil.apply(poseBlue);
  }

  /**
   * Returns the angle the robot should face to point at the alliance hub.
   *
   * <p>This calculates the bearing from the robot's current position to the alliance hub target.
   * The returned angle is automatically normalized to choose the shortest rotation path.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Gets the robot's current position
   *   <li>Gets the alliance hub target position (auto-flips for red alliance)
   *   <li>Calculates the angle from robot to hub using atan2
   *   <li>Returns a Rotation2d that can be passed to joystickDriveAtAngle
   * </ol>
   *
   * <p><b>Continuous Input Handling:</b> The ProfiledPIDController in joystickDriveAtAngle has
   * continuous input enabled (-π to π), so it will automatically choose the shortest rotation path.
   * For example, if the robot is at 170° and the hub is at -170°, it will rotate 20° instead of
   * 340°.
   *
   * <p><b>Usage:</b>
   *
   * <pre>
   * // In a command - continuously update heading to point at hub
   * DriveCommands.joystickDriveAtAngle(
   *   drive,
   *   xSupplier,
   *   ySupplier,
   *   () -> RobotState.getInstance().getAngleToAllianceHub()
   * );
   * </pre>
   *
   * @return The heading the robot should face to point the front of the robot at the alliance hub
   */
  public Rotation2d getAngleToAllianceHub() {
    Rotation2d angle = getAngleToAllianceHub(false);
    Logger.recordOutput("RobotState/AngleToAllianceHub", angle);
    return angle;
  }

  /**
   * Returns the angle the robot should face to point at the alliance hub.
   *
   * <p>When {@code useBack} is true, the returned angle is rotated 180° so the <b>back</b> of the
   * robot faces the hub. This is useful when the shooter is mounted on the back of the robot and
   * needs to face the target while the driver drives "forward" away from the hub.
   *
   * @param useBack If true, aims the back of the robot at the hub (e.g. for a rear-mounted
   *     shooter). If false, aims the front of the robot at the hub.
   * @return The heading the robot should face to point the desired end at the alliance hub
   */
  public Rotation2d getAngleToAllianceHub(boolean useBack) {
    // Get current robot position
    Pose2d currentPose = getEstimatedPose();

    // Get alliance hub target (2D position on the field)
    Translation3d hubTarget3d = getAllianceHubTarget();
    Translation2d hubTarget2d = hubTarget3d.toTranslation2d();

    // Calculate the vector from robot to hub
    Translation2d robotToHub = hubTarget2d.minus(currentPose.getTranslation());

    // Calculate the angle using atan2
    // This gives us the direction we need to face to point at the hub
    Rotation2d angle = new Rotation2d(robotToHub.getX(), robotToHub.getY());

    // If useBack is true, rotate 180° so the back of the robot faces the hub.
    // This is needed when the shooter is rear-mounted.
    if (useBack) {
      angle = angle.plus(Rotation2d.kPi);
    }

    return angle;
  }

  // ==================== ODOMETRY UPDATES ====================

  /**
   * Adds an odometry observation to update the pose estimate.
   *
   * <p>This should be called by the Drive subsystem every loop with the latest gyro reading and
   * module positions.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Calculates how far each wheel has moved since last update
   *   <li>Combines wheel movements with gyro rotation
   *   <li>Updates the pose estimator with the new data
   * </ol>
   *
   * @param gyroRotation The current gyro rotation
   * @param modulePositions The current module positions (distance and angle for each module)
   * @param moduleStates The current module states (velocity and angle for each module)
   */
  public void addOdometryObservation(
      Rotation2d gyroRotation,
      SwerveModulePosition[] modulePositions,
      SwerveModuleState[] moduleStates) {

    // Store for next iteration
    rawGyroRotation = gyroRotation;
    lastModulePositions = modulePositions;
    currentModuleStates = moduleStates;

    // Update pose estimator
    poseEstimator.update(gyroRotation, modulePositions);

    // Log the update
    Logger.recordOutput("RobotState/GyroRotation", gyroRotation);
  }

  /**
   * Adds an odometry observation with a specific timestamp.
   *
   * <p>This version is used when processing high-frequency odometry samples that were captured at
   * different times during a single loop.
   *
   * @param timestamp The timestamp when the observation was captured (seconds)
   * @param gyroRotation The gyro rotation at that timestamp
   * @param modulePositions The module positions at that timestamp
   * @param moduleStates The module states for velocity calculation
   */
  public void addOdometryObservation(
      double timestamp,
      Rotation2d gyroRotation,
      SwerveModulePosition[] modulePositions,
      SwerveModuleState[] moduleStates) {

    // Store for next iteration
    rawGyroRotation = gyroRotation;
    lastModulePositions = modulePositions;
    currentModuleStates = moduleStates;

    // Update pose estimator with timestamp
    poseEstimator.updateWithTime(timestamp, gyroRotation, modulePositions);
  }

  /**
   * Adds a vision measurement to improve the pose estimate.
   *
   * <p>Vision measurements help correct for odometry drift. The standard deviations control how
   * much the vision data is trusted compared to odometry.
   *
   * @param visionPose The pose measured by vision
   * @param timestampSeconds When the measurement was captured
   * @param stdDevs Standard deviations for x, y, and rotation (smaller = more trust)
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  /**
   * Resets the pose estimator to a specific pose.
   *
   * <p>Use this to set the robot's starting position at the beginning of autonomous.
   *
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }

  // ==================== TARGET ENUM ====================

  /**
   * Defines the possible shooting targets on the field.
   *
   * <p>Each target has a 3D position defined from the <b>blue alliance perspective</b>. {@link
   * AllianceFlipUtil#apply(Translation3d)} is used at runtime to mirror positions for red alliance.
   *
   * <p><b>Targets:</b>
   *
   * <ul>
   *   <li>{@link #HUB} — The alliance hub (primary scoring target)
   *   <li>{@link #FEED_LEFT} — Feed shot toward the left side of the field (from driver POV)
   *   <li>{@link #FEED_RIGHT} — Feed shot toward the right side of the field (from driver POV)
   * </ul>
   */
  public enum Target {
    /** The alliance hub — primary scoring target. */
    HUB(
        new Translation3d(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.topCenterPoint.getY(),
            FieldConstants.Hub.height)),

    /**
     * Feed shot toward the left side of the field (from the blue driver station POV).
     *
     * <p>Position is halfway between the neutral zone line and the alliance wall, on the left half
     * of the field. Height is 0 because feed shots are lobbed to the ground.
     */
    FEED_LEFT(
        new Translation3d(
            FieldConstants.LinesVertical.neutralZoneNear / 2.0,
            (FieldConstants.LinesHorizontal.center + FieldConstants.fieldWidth) / 2.0,
            0)),

    /**
     * Feed shot toward the right side of the field (from the blue driver station POV).
     *
     * <p>Position is halfway between the neutral zone line and the alliance wall, on the right half
     * of the field.
     */
    FEED_RIGHT(
        new Translation3d(
            FieldConstants.LinesVertical.neutralZoneNear / 2.0,
            (0.0 + FieldConstants.LinesHorizontal.center) / 2.0,
            0));

    /** The 3D position of this target from the blue alliance perspective. */
    private final Translation3d bluePosition;

    Target(Translation3d bluePosition) {
      this.bluePosition = bluePosition;
    }

    /**
     * Returns the alliance-corrected 3D position of this target.
     *
     * <p>Automatically mirrors the position for red alliance using {@link AllianceFlipUtil}.
     *
     * @return The target position in field coordinates
     */
    public Translation3d getPosition() {
      return AllianceFlipUtil.apply(bluePosition);
    }

    /**
     * Returns whether this target is a feed shot (FEED_LEFT or FEED_RIGHT).
     *
     * @return true if this is a feed target, false if it's the HUB
     */
    public boolean isFeedTarget() {
      return this == FEED_LEFT || this == FEED_RIGHT;
    }
  }

  // ==================== FIELD REGION ENUM ====================

  /**
   * Regions of the field used to determine which target the robot should shoot at.
   *
   * <p>All regions are defined from the <b>blue alliance perspective</b> and are automatically
   * flipped for red alliance at runtime using {@link AllianceFlipUtil}.
   *
   * <p><b>Target Selection Rules:</b>
   *
   * <ul>
   *   <li>{@link #ALLIANCE_ZONE} → Shoot at {@link Target#HUB}
   *   <li>{@link #LEFT_BUMP_TRENCH} → Shoot at {@link Target#HUB}
   *   <li>{@link #RIGHT_BUMP_TRENCH} → Shoot at {@link Target#HUB}
   *   <li>{@link #NEUTRAL_ZONE} → Feed shot (left or right based on robot Y position)
   *   <li>{@link #OPPONENT_ALLIANCE_ZONE} → Feed shot (left or right based on robot Y position)
   * </ul>
   */
  public enum FieldRegion {
    /** The area near our alliance wall, between the wall and the neutral zone line. */
    ALLIANCE_ZONE,

    /** The center of the field between the two neutral zone lines. */
    NEUTRAL_ZONE,

    /** The area near the opposing alliance wall. */
    OPPONENT_ALLIANCE_ZONE,

    /** The trench on the left side of the hub (from blue driver station POV). */
    LEFT_BUMP_TRENCH,

    /** The trench on the right side of the hub (from blue driver station POV). */
    RIGHT_BUMP_TRENCH
  }

  // ==================== FIELD REGION DETECTION ====================

  /**
   * Determines which region of the field the robot is currently in.
   *
   * <p>Uses polygon-based hit detection with {@link PointInPolygon} to classify the robot's
   * position. The polygons are defined from the blue alliance perspective and the robot's position
   * is un-flipped before testing so the same polygons work for both alliances.
   *
   * <p><b>Region priority:</b> Trenches are checked first since they overlap with other zones.
   *
   * @return The {@link FieldRegion} the robot is currently in
   */
  public FieldRegion getFieldRegion() {
    // Un-flip the robot's position to blue-alliance coordinates so we can use
    // a single set of polygon definitions for both alliances.
    Translation2d robotPos = AllianceFlipUtil.apply(getEstimatedPose().getTranslation());

    // Check trenches first (they are sub-regions of the alliance zone)
    if (isInLeftBumpTrench(robotPos)) {
      Logger.recordOutput("RobotState/FieldRegion", FieldRegion.LEFT_BUMP_TRENCH.name());
      return FieldRegion.LEFT_BUMP_TRENCH;
    }
    if (isInRightBumpTrench(robotPos)) {
      Logger.recordOutput("RobotState/FieldRegion", FieldRegion.RIGHT_BUMP_TRENCH.name());
      return FieldRegion.RIGHT_BUMP_TRENCH;
    }

    // Check main zones by X position (blue alliance perspective)
    double x = robotPos.getX();
    FieldRegion region;
    if (x <= FieldConstants.LinesVertical.neutralZoneNear) {
      region = FieldRegion.ALLIANCE_ZONE;
    } else if (x <= FieldConstants.LinesVertical.neutralZoneFar) {
      region = FieldRegion.NEUTRAL_ZONE;
    } else {
      region = FieldRegion.OPPONENT_ALLIANCE_ZONE;
    }
    Logger.recordOutput("RobotState/FieldRegion", region.name());
    return region;
  }

  /**
   * Checks if a point (in blue-alliance coordinates) is inside the left bump trench.
   *
   * <p>The left bump trench polygon is defined by the four corners of the left bump field element.
   *
   * @param point The robot's position in blue-alliance coordinates
   * @return true if the robot is in the left bump trench
   */
  private boolean isInLeftBumpTrench(Translation2d point) {
    // Define the left bump trench polygon using the bump corner constants
    List<Translation2d> polygon =
        List.of(
            FieldConstants.LeftBump.nearLeftCorner,
            FieldConstants.LeftBump.nearRightCorner,
            FieldConstants.LeftBump.farRightCorner,
            FieldConstants.LeftBump.farLeftCorner);
    return PointInPolygon.pointInPolygon(point, polygon);
  }

  /**
   * Checks if a point (in blue-alliance coordinates) is inside the right bump trench.
   *
   * @param point The robot's position in blue-alliance coordinates
   * @return true if the robot is in the right bump trench
   */
  private boolean isInRightBumpTrench(Translation2d point) {
    // Define the right bump trench polygon using the bump corner constants
    List<Translation2d> polygon =
        List.of(
            FieldConstants.RightBump.nearLeftCorner,
            FieldConstants.RightBump.nearRightCorner,
            FieldConstants.RightBump.farRightCorner,
            FieldConstants.RightBump.farLeftCorner);
    return PointInPolygon.pointInPolygon(point, polygon);
  }

  // ==================== TARGET SELECTION ====================

  /**
   * Returns the target the robot should currently shoot at based on its field position.
   *
   * <p><b>Logic:</b>
   *
   * <ul>
   *   <li>Alliance zone or either trench → {@link Target#HUB}
   *   <li>Neutral zone or opponent zone → Feed shot (left or right based on robot Y)
   * </ul>
   *
   * <p>For feed targets, the side is determined by comparing the robot's Y position to the field
   * center line. If the robot is on the left half of the field (from the blue driver station POV),
   * it feeds left; otherwise it feeds right.
   *
   * @return The {@link Target} the robot should shoot at
   */
  public Target getActiveTarget() {
    FieldRegion region = getFieldRegion();

    Target target;
    switch (region) {
      case ALLIANCE_ZONE:
      case LEFT_BUMP_TRENCH:
      case RIGHT_BUMP_TRENCH:
        // Close to our hub — shoot directly at it
        target = Target.HUB;
        break;

      case NEUTRAL_ZONE:
      case OPPONENT_ALLIANCE_ZONE:
      default:
        // Too far from hub — lob a feed shot to our alliance side
        target = getFeedTargetForCurrentPosition();
        break;
    }
    Logger.recordOutput("RobotState/ActiveTarget", target.name());
    return target;
  }

  /**
   * Determines which feed target (left or right) to use based on the robot's Y position.
   *
   * <p>Compares the robot's Y coordinate to the field center line. This uses the actual
   * (non-flipped) robot position because the feed targets are already alliance-flipped.
   *
   * @return {@link Target#FEED_LEFT} or {@link Target#FEED_RIGHT}
   */
  private Target getFeedTargetForCurrentPosition() {
    // Un-flip to blue coordinates for consistent left/right determination
    Translation2d bluePos = AllianceFlipUtil.apply(getEstimatedPose().getTranslation());
    double fieldCenterY = FieldConstants.fieldWidth / 2.0;

    if (bluePos.getY() > fieldCenterY) {
      return Target.FEED_LEFT;
    } else {
      return Target.FEED_RIGHT;
    }
  }

  // ==================== ANGLE TO TARGET ====================

  /**
   * Returns the angle the robot should face to point at the currently active target.
   *
   * <p>This is the primary method commands should use for auto-aiming. It automatically selects the
   * correct target based on field position.
   *
   * @return The heading to face the active target
   */
  public Rotation2d getAngleToActiveTarget() {
    Rotation2d angle = getAngleToTarget(getActiveTarget());
    Logger.recordOutput("RobotState/AngleToActiveTarget", angle);
    return angle;
  }

  /**
   * Returns the angle the robot should face to point at a specific target.
   *
   * <p>Calculates the bearing from the robot's current position to the target's position.
   *
   * @param target The target to aim at
   * @return The heading the robot should face to point at the target
   */
  public Rotation2d getAngleToTarget(Target target) {
    Pose2d currentPose = getEstimatedPose();
    Translation2d targetPos = target.getPosition().toTranslation2d();
    Translation2d robotToTarget = targetPos.minus(currentPose.getTranslation());

    // Log the target position and vector for debugging aim issues
    Logger.recordOutput("RobotState/TargetPosition", targetPos);
    Logger.recordOutput("RobotState/RobotToTargetVector", robotToTarget);

    return new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
  }

  /**
   * Returns the distance from the robot to the currently active target.
   *
   * @return Distance to the active target
   */
  public Distance getDistanceToActiveTarget() {
    Distance distance = getDistanceToTarget(getActiveTarget());
    Logger.recordOutput("RobotState/DistanceToActiveTarget_m", distance.in(Meters));
    return distance;
  }

  /**
   * Returns the distance from the robot to a specific target.
   *
   * @param target The target to measure distance to
   * @return Distance to the target
   */
  public Distance getDistanceToTarget(Target target) {
    return getDistanceToPoint(target.getPosition().toTranslation2d());
  }

  // ==================== TRIGGERS ====================

  /**
   * Tolerance (in degrees) for determining if the robot is facing its target.
   *
   * <p>Used by the {@link #facingTarget} trigger.
   */
  private static final double SHOOT_TOLERANCE_DEGREES = 5.0;

  /**
   * Trigger that fires when the robot is facing the active target within tolerance.
   *
   * <p>This can be used to gate shooting — only feed the game piece when the robot is aimed
   * correctly. Checks if the difference between the robot's current heading and the angle to the
   * target is within {@link #SHOOT_TOLERANCE_DEGREES}.
   *
   * <p><b>Usage:</b>
   *
   * <pre>
   * RobotState.getInstance().facingTarget
   *     .and(() -> shooter.areFlywheelsAtTargetSpeed())
   *     .onTrue(FeederCommands.runFeeder(feeder));
   * </pre>
   */
  public final LoggedTrigger facingTarget =
      new LoggedTrigger(
          "RobotState/FacingTarget",
          () -> {
            Rotation2d targetAngle = getAngleToActiveTarget();
            Rotation2d robotHeading = getEstimatedPose().getRotation();
            double headingErrorDeg = targetAngle.minus(robotHeading).getDegrees();

            // Log intermediate values so we can debug aiming in AdvantageScope
            Logger.recordOutput("RobotState/FacingTarget/TargetAngleDeg", targetAngle.getDegrees());
            Logger.recordOutput(
                "RobotState/FacingTarget/RobotHeadingDeg", robotHeading.getDegrees());
            Logger.recordOutput("RobotState/FacingTarget/HeadingErrorDeg", headingErrorDeg);
            Logger.recordOutput("RobotState/FacingTarget/ToleranceDeg", SHOOT_TOLERANCE_DEGREES);

            return Math.abs(headingErrorDeg) < SHOOT_TOLERANCE_DEGREES;
          });

  // ==================== TRENCH TRIGGERS (for robots with adjustable hoods) ====================

  /**
   * Maximum time (in seconds) to look ahead when predicting trench entry.
   *
   * <p>If the robot will enter a trench within this time window, the {@link #enteringTrench}
   * trigger fires. This gives mechanisms time to retract before hitting the trench ceiling.
   */
  private static final double MAX_TRENCH_RETRACT_TIME = 0.2;

  /**
   * Trigger that fires when the robot is about to enter a bump trench.
   *
   * <p><b>Purpose:</b> On robots with an adjustable hood or tall mechanisms, this trigger can be
   * used to force retraction before the robot drives under the trench ceiling (22" clearance).
   *
   * <p><b>How it works:</b> Projects the robot's current position forward using its velocity and
   * checks if the projected position would be inside a trench within {@link
   * #MAX_TRENCH_RETRACT_TIME} seconds.
   *
   * <p><b>Note:</b> This trigger is defined but NOT used by the current robot (fixed hood). It is
   * provided for reference and for robots that need trench-aware hood retraction.
   *
   * <p><b>Usage (for a robot with an adjustable hood):</b>
   *
   * <pre>
   * RobotState.getInstance().enteringTrench
   *     .onTrue(HoodCommands.retractHood(hood));
   * </pre>
   */
  public final Trigger enteringTrench =
      new Trigger(
          () -> {
            // Get the robot's current velocity in field-relative coordinates
            ChassisSpeeds fieldVelocity = getFieldRelativeVelocity();
            double vx = fieldVelocity.vxMetersPerSecond;
            double vy = fieldVelocity.vyMetersPerSecond;
            double speed = MetersPerSecond.of(Math.hypot(vx, vy)).in(MetersPerSecond);

            // If the robot is barely moving, it's not entering anything
            if (speed < 0.1) {
              return false;
            }

            // Project the robot's position forward by the retract time
            Translation2d currentPos = getEstimatedPose().getTranslation();
            Translation2d projectedPos =
                currentPos.plus(
                    new Translation2d(vx * MAX_TRENCH_RETRACT_TIME, vy * MAX_TRENCH_RETRACT_TIME));

            // Un-flip to blue coordinates for polygon check
            Translation2d blueProjected = AllianceFlipUtil.apply(projectedPos);

            // Check if the projected position is inside either trench
            return isInLeftBumpTrench(blueProjected) || isInRightBumpTrench(blueProjected);
          });

  /**
   * Trigger that fires when the hood is safe to actuate for a hub shot in autonomous.
   *
   * <p>The hood is considered safe when:
   *
   * <ul>
   *   <li>The robot is in the alliance zone (near our hub)
   *   <li>The robot is NOT about to enter a trench
   * </ul>
   *
   * <p><b>Note:</b> This trigger is defined but NOT used by the current robot (fixed hood). It is
   * provided for reference and for robots with adjustable hoods that need trench awareness.
   *
   * <p><b>Usage (for a robot with an adjustable hood):</b>
   *
   * <pre>
   * RobotState.getInstance().hoodSafe
   *     .onTrue(HoodCommands.setHoodForDistance(hood));
   * </pre>
   */
  public final Trigger hoodSafe =
      new Trigger(
          () ->
              getFieldRegion() == FieldRegion.ALLIANCE_ZONE
                  && enteringTrench.negate().getAsBoolean());
}
