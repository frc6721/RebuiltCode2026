package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.Target;
import frc.robot.util.PointInPolygon;
import java.util.List;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates the required flywheel speed to hit a target on the field.
 *
 * <p>This is a singleton class that handles all shot calculations for the shooter. It uses the
 * centralized RobotState to get the robot's current position and calculates the distance to the
 * target.
 *
 * <p><b>How Distance-Based Shooting Works:</b>
 *
 * <ol>
 *   <li>Get robot's current position from RobotState
 *   <li>Calculate 2D distance to the target (horizontal distance)
 *   <li>Look up required flywheel RPM from the interpolation table
 *   <li>The table automatically interpolates between characterization data points
 * </ol>
 *
 * <p><b>Characterization:</b> To use this system effectively, you need to characterize your shooter
 * by:
 *
 * <ol>
 *   <li>Placing the robot at known distances from the target
 *   <li>Adjusting RPM until shots consistently score
 *   <li>Recording the (distance, RPM) pairs in ShooterConstants.DistanceMap.SPEED_MAP
 * </ol>
 *
 * <p><b>Future Enhancements:</b> This class is designed to be extended for shooting-while-moving.
 * The structure allows for:
 *
 * <ul>
 *   <li>Robot velocity compensation
 *   <li>Lead time calculations
 * </ul>
 *
 * <p><b>Usage:</b>
 *
 * <pre>
 * // Get the speed needed to hit the alliance hub
 * Translation3d hubTarget = RobotState.getInstance().getAllianceHubTarget();
 * AngularVelocity speed = ShotCalculator.getInstance().getFlywheelSpeedForTarget(hubTarget);
 *
 * // Or use the convenience method
 * AngularVelocity speed = ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub();
 * </pre>
 */
public class ShotCalculator {

  private static ShotCalculator instance;

  /**
   * Returns the singleton instance of ShotCalculator.
   *
   * @return The ShotCalculator singleton
   */
  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  /** Private constructor - use getInstance() instead. */
  private ShotCalculator() {}

  // ==================== SHOT CALCULATION ====================

  /**
   * Calculates the required flywheel speed to hit a target.
   *
   * <p>This method:
   *
   * <ol>
   *   <li>Gets the robot's current position from RobotState
   *   <li>Calculates the 2D (horizontal) distance to the target
   *   <li>Looks up the required RPM from the interpolation table
   *   <li>Clamps the result to safe min/max speeds
   * </ol>
   *
   * <p><b>Note:</b> Currently uses 2D distance only. The target's Z coordinate (height) is used for
   * logging but doesn't affect the calculation. This could be enhanced for trajectory-based
   * shooting.
   *
   * @param target The target position to shoot at (Translation3d in field coordinates)
   * @return The required flywheel angular velocity
   */
  public AngularVelocity getFlywheelSpeedForTarget(Translation3d target) {
    return getFlywheelSpeedForTarget(target, ShooterConstants.DistanceMap.SPEED_MAP);
  }

  /**
   * Calculates the required flywheel speed using a specific speed map.
   *
   * <p>Different targets (hub vs feed) use different speed maps because they require different RPMs
   * at the same distance. Hub shots are direct and fast; feed shots are slower lobs.
   *
   * @param target The target position to shoot at (Translation3d in field coordinates)
   * @param speedMap The interpolation table mapping distance (meters) to RPM
   * @return The required flywheel angular velocity
   */
  public AngularVelocity getFlywheelSpeedForTarget(
      Translation3d target, InterpolatingDoubleTreeMap speedMap) {
    Translation2d robotPosition = RobotState.getInstance().getEstimatedPose().getTranslation();

    Translation2d targetPosition2d = target.toTranslation2d();
    double distanceMeters = robotPosition.getDistance(targetPosition2d);

    double speedRPM = speedMap.get(distanceMeters);

    double minRPM = ShooterConstants.Limits.MIN_SPEED.in(RPM);
    double maxRPM = ShooterConstants.Limits.MAX_SPEED.in(RPM);
    speedRPM = Math.max(minRPM, Math.min(maxRPM, speedRPM));

    Logger.recordOutput("Shooter/ShotCalculator/TargetPosition", target);
    Logger.recordOutput("Shooter/ShotCalculator/RobotPosition", robotPosition);
    Logger.recordOutput("Shooter/ShotCalculator/Distance_m", distanceMeters);
    Logger.recordOutput("Shooter/ShotCalculator/CalculatedSpeed_RPM", speedRPM);

    return RPM.of(speedRPM);
  }

  /**
   * Calculates the required flywheel speed for a specific {@link Target} enum value.
   *
   * <p>Automatically selects the correct speed map:
   *
   * <ul>
   *   <li>{@link Target#HUB} → Uses the hub speed map ({@link
   *       ShooterConstants.DistanceMap#SPEED_MAP})
   *   <li>{@link Target#FEED_LEFT} / {@link Target#FEED_RIGHT} → Uses the feed speed map ({@link
   *       ShooterConstants.DistanceMap#FEED_SPEED_MAP})
   * </ul>
   *
   * @param target The target to shoot at
   * @return The required flywheel angular velocity
   */
  public AngularVelocity getFlywheelSpeedForTarget(Target target) {
    InterpolatingDoubleTreeMap speedMap =
        target.isFeedTarget()
            ? ShooterConstants.DistanceMap.FEED_SPEED_MAP
            : ShooterConstants.DistanceMap.SPEED_MAP;

    Logger.recordOutput("Shooter/ShotCalculator/TargetType", target.name());
    Logger.recordOutput("Shooter/ShotCalculator/IsFeedShot", target.isFeedTarget());

    return getFlywheelSpeedForTarget(target.getPosition(), speedMap);
  }

  /**
   * Calculates the required flywheel speed for the currently active target.
   *
   * <p>This is the primary method commands should use. It automatically:
   *
   * <ol>
   *   <li>Determines the active target from {@link RobotState#getActiveTarget()}
   *   <li>Selects the correct speed map (hub vs feed)
   *   <li>Calculates distance-based RPM
   * </ol>
   *
   * @return The required flywheel angular velocity for the active target
   */
  public AngularVelocity getFlywheelSpeedForActiveTarget() {
    Target activeTarget = RobotState.getInstance().getActiveTarget();
    return getFlywheelSpeedForTarget(activeTarget);
  }

  /**
   * Calculates the required flywheel speed to hit the alliance hub.
   *
   * <p>This is a convenience method that automatically uses the correct hub target based on
   * alliance color (handled by RobotState).
   *
   * @return The required flywheel angular velocity to hit the alliance hub
   */
  public AngularVelocity getFlywheelSpeedForAllianceHub() {
    Translation3d hubTarget = RobotState.getInstance().getAllianceHubTarget();
    return getFlywheelSpeedForTarget(hubTarget);
  }

  /**
   * Returns the 2D distance from the robot to a target.
   *
   * <p>Useful for logging and decision-making (e.g., "should we even try to shoot from here?")
   *
   * @param target The target position
   * @return The horizontal distance to the target
   */
  public Distance getDistanceToTarget(Translation3d target) {
    Translation2d robotPosition = RobotState.getInstance().getEstimatedPose().getTranslation();
    Translation2d targetPosition2d = target.toTranslation2d();
    double distanceMeters = robotPosition.getDistance(targetPosition2d);
    return Meters.of(distanceMeters);
  }

  /**
   * Checks if the robot is within effective shooting range of the alliance hub.
   *
   * <p>This can be used to provide driver feedback (LEDs, rumble) or to gate shooting commands.
   *
   * <p>The effective range is determined by the data points in the interpolation table - shooting
   * from distances outside the characterized range may be inaccurate.
   *
   * @return true if the robot is within effective shooting range
   */
  public boolean isInShootingRange() {
    Distance distance = RobotState.getInstance().getDistanceToAllianceHub();
    double distanceMeters = distance.in(Meters);

    // Check if within the characterized range
    // The DISTANCE_TO_SPEED_MAP defines the range we've tested
    boolean inRange = distanceMeters >= 0.5 && distanceMeters <= 15.0;

    Logger.recordOutput("Shooter/ShotCalculator/InShootingRange", inRange);
    return inRange;
  }

  // ==================== TUNED DISTANCE SHOOTING ====================

  /**
   * Finds the closest tuned hub shot distance from the robot's current position.
   *
   * <p>Iterates through all distances in {@link
   * ShooterConstants.DistanceMap#TUNED_HUB_SHOT_DISTANCES_METERS} and returns the one closest to
   * the robot's current distance from the hub. Before returning, it verifies that driving to that
   * distance (on the line between the robot and hub) would not put the robot inside the alliance
   * tower's collision footprint.
   *
   * <p>If the closest distance would collide with the tower, the next-closest distance is tried,
   * and so on. If NO safe distance is found, an empty optional is returned — callers should treat
   * this as "shooting is blocked" and not attempt to feed.
   *
   * @return The closest safe tuned distance from the hub (in meters), or empty if all distances
   *     collide with the tower
   */
  public OptionalDouble getClosestTunedHubDistanceMeters() {
    double[] tunedDistances = ShooterConstants.DistanceMap.TUNED_HUB_SHOT_DISTANCES_METERS;
    if (tunedDistances.length == 0) {
      // No tuned distances configured — return empty (cannot auto-distance)
      Logger.recordOutput("Shooter/ShotCalculator/AutoDistanceBlocked", true);
      return OptionalDouble.empty();
    }

    Translation2d robotPos = RobotState.getInstance().getEstimatedPose().getTranslation();
    Translation2d hubPos = RobotState.getInstance().getAllianceHubTarget().toTranslation2d();
    double currentDistance = robotPos.getDistance(hubPos);

    // Sort indices by how close each tuned distance is to the current distance
    // (closest first). We'll try them in order and pick the first one that's safe.
    int[] sortedIndices = new int[tunedDistances.length];
    for (int i = 0; i < sortedIndices.length; i++) {
      sortedIndices[i] = i;
    }
    // Simple selection sort (array is small, no need for fancy sorting)
    for (int i = 0; i < sortedIndices.length - 1; i++) {
      for (int j = i + 1; j < sortedIndices.length; j++) {
        double diffI = Math.abs(tunedDistances[sortedIndices[i]] - currentDistance);
        double diffJ = Math.abs(tunedDistances[sortedIndices[j]] - currentDistance);
        if (diffJ < diffI) {
          int temp = sortedIndices[i];
          sortedIndices[i] = sortedIndices[j];
          sortedIndices[j] = temp;
        }
      }
    }

    // Try each tuned distance from closest to farthest
    for (int idx : sortedIndices) {
      double tunedDistance = tunedDistances[idx];
      Translation2d candidatePosition =
          getPositionAtDistanceFromHub(robotPos, hubPos, tunedDistance);

      if (!isInsideTowerFootprint(candidatePosition)) {
        Logger.recordOutput("Shooter/ShotCalculator/ClosestTunedDistance_m", tunedDistance);
        Logger.recordOutput("Shooter/ShotCalculator/AutoDriveTarget", candidatePosition);
        Logger.recordOutput("Shooter/ShotCalculator/AutoDistanceBlocked", false);
        return OptionalDouble.of(tunedDistance);
      }
    }

    // All tuned distances collide with tower — shooting is blocked
    Logger.recordOutput("Shooter/ShotCalculator/AutoDistanceBlocked", true);
    return OptionalDouble.empty();
  }

  /**
   * Calculates the field position that is a specific distance from the hub, along the line from the
   * hub center through the robot's current position.
   *
   * <p>This is used to determine where the robot would end up if it drove straight toward or away
   * from the hub to reach a specific shooting distance.
   *
   * @param robotPos The robot's current 2D position
   * @param hubPos The hub's 2D position (center)
   * @param targetDistance The desired distance from the hub center (meters)
   * @return The 2D position on the field at the desired distance from the hub
   */
  public Translation2d getPositionAtDistanceFromHub(
      Translation2d robotPos, Translation2d hubPos, double targetDistance) {
    // Vector from hub to robot
    Translation2d hubToRobot = robotPos.minus(hubPos);
    double currentDistance = hubToRobot.getNorm();

    if (currentDistance < 0.001) {
      // Robot is essentially on top of the hub — pick an arbitrary direction
      // (toward the alliance wall, negative X from hub)
      return new Translation2d(hubPos.getX() - targetDistance, hubPos.getY());
    }

    // Normalize the direction vector and scale to the desired distance
    double scale = targetDistance / currentDistance;
    Translation2d offset = new Translation2d(hubToRobot.getX() * scale, hubToRobot.getY() * scale);
    return hubPos.plus(offset);
  }

  /**
   * Checks if a candidate position falls inside the alliance tower's collision footprint.
   *
   * <p>The tower footprint includes a safety margin defined in {@link
   * FieldConstants.Tower#COLLISION_MARGIN}. The check is done in blue-alliance coordinates so a
   * single polygon definition works for both alliances.
   *
   * @param fieldPosition The candidate position in field coordinates
   * @return true if the position is inside the tower footprint (unsafe)
   */
  private boolean isInsideTowerFootprint(Translation2d fieldPosition) {
    // Convert to blue-alliance coordinates for consistent polygon check
    Translation2d bluePos = AllianceFlipUtil.apply(fieldPosition);
    List<Translation2d> towerFootprint = FieldConstants.Tower.getAllianceTowerFootprint();
    return PointInPolygon.pointInPolygon(bluePos, towerFootprint);
  }

  /**
   * Returns the 2D field position the robot should drive to for the closest tuned hub shot.
   *
   * <p>This is a convenience method that combines {@link #getClosestTunedHubDistanceMeters()} with
   * {@link #getPositionAtDistanceFromHub} to give a concrete drive target.
   *
   * <p>If no safe tuned distance exists (all positions collide with the tower), returns an empty
   * optional. Callers should not apply radial correction when empty.
   *
   * @return The target position on the field, or empty if all tuned distances are blocked
   */
  public java.util.Optional<Translation2d> getAutoDistanceDriveTarget() {
    OptionalDouble tunedDistance = getClosestTunedHubDistanceMeters();
    if (tunedDistance.isEmpty()) {
      return java.util.Optional.empty();
    }
    Translation2d robotPos = RobotState.getInstance().getEstimatedPose().getTranslation();
    Translation2d hubPos = RobotState.getInstance().getAllianceHubTarget().toTranslation2d();
    return java.util.Optional.of(
        getPositionAtDistanceFromHub(robotPos, hubPos, tunedDistance.getAsDouble()));
  }

  /**
   * Returns whether the auto-distance system is blocked because every tuned shot position would put
   * the robot inside the alliance tower's collision footprint.
   *
   * <p>This is intended for use by the Shooter subsystem and commands to determine if shooting
   * should be prevented when using the auto-distance feature.
   *
   * @return true if no safe tuned distance exists (all collide with tower)
   */
  public boolean isAutoDistanceBlocked() {
    return getClosestTunedHubDistanceMeters().isEmpty();
  }
}
