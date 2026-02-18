package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizes the intake pivot mechanism using Mechanism2d and logs a Pose3d for AdvantageScope 3D
 * visualization.
 *
 * <p>The visualization shows:
 *
 * <ul>
 *   <li><b>Green arm:</b> Current measured position
 *   <li><b>Yellow arm:</b> Trajectory/setpoint position (where PID is targeting)
 *   <li><b>Red arm:</b> Goal position (final target)
 *   <li><b>White lines:</b> Min and max angle limits
 *   <li><b>Background:</b> Green when at goal, black otherwise
 * </ul>
 */
public class IntakeVisualizer {

  // Mechanism2d canvas dimensions
  private static final double WIDTH = 1.0;
  private static final double HEIGHT = 1.0;

  // Visualization components
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d measuredArm;
  private final LoggedMechanismLigament2d setpointArm;
  private final LoggedMechanismLigament2d goalArm;
  private final LoggedMechanismLigament2d lowerBound;
  private final LoggedMechanismLigament2d upperBound;

  // Configuration
  private final String name;
  private final double armLength;
  private final Pose3d baseOffset;

  /**
   * Creates a new IntakeVisualizer.
   *
   * @param name The name for logging (e.g., "Intake")
   */
  public IntakeVisualizer(String name) {
    this.name = name;
    this.armLength = IntakeConstants.Visualization.ARM_LENGTH;
    this.baseOffset =
        new Pose3d(IntakeConstants.Visualization.OFFSET, IntakeConstants.Visualization.ROTATION);

    // Create the Mechanism2d canvas
    mechanism = new LoggedMechanism2d(WIDTH, HEIGHT, new Color8Bit(Color.kBlack));

    // Create the root at center of canvas
    LoggedMechanismRoot2d root =
        mechanism.getRoot(name + "_Root", baseOffset.getX(), baseOffset.getY());

    // Create min/max angle boundary indicators
    // Transform from robot frame to Mechanism2d frame
    double minAngleMech2d =
        90.0 - IntakeConstants.Mechanical.MAX_ANGLE_DEGREES; // Note: min robot = max visual
    double maxAngleMech2d =
        90.0 - IntakeConstants.Mechanical.MIN_ANGLE_DEGREES; // Note: max robot = min visual

    lowerBound =
        new LoggedMechanismLigament2d(
            name + "_LowerBound", armLength, minAngleMech2d, 2, new Color8Bit(Color.kWhite));

    upperBound =
        new LoggedMechanismLigament2d(
            name + "_UpperBound", armLength, maxAngleMech2d, 2, new Color8Bit(Color.kWhite));

    // Create the measured position arm (what the encoder reads)
    measuredArm =
        new LoggedMechanismLigament2d(
            name + "_Measured",
            armLength,
            90.0
                - IntakeConstants.Mechanical
                    .STARTING_ANGLE_DEGREES, // Transform to Mechanism2d frame
            6,
            new Color8Bit(Color.kGreen));

    // Create the setpoint arm (where PID is currently targeting)
    setpointArm =
        new LoggedMechanismLigament2d(
            name + "_Setpoint",
            armLength * 0.9, // Slightly shorter to see both
            90.0
                - IntakeConstants.Mechanical
                    .STARTING_ANGLE_DEGREES, // Transform to Mechanism2d frame
            4,
            new Color8Bit(Color.kYellow));

    // Create the goal arm (final target position)
    goalArm =
        new LoggedMechanismLigament2d(
            name + "_Goal",
            armLength * 0.8, // Even shorter to see all three
            90.0
                - IntakeConstants.Mechanical
                    .STARTING_ANGLE_DEGREES, // Transform to Mechanism2d frame
            3,
            new Color8Bit(Color.kRed));

    // Attach all ligaments to the root
    root.append(lowerBound);
    root.append(upperBound);
    root.append(measuredArm);
    root.append(setpointArm);
    root.append(goalArm);
  }

  /**
   * Updates the visualizer with the current state.
   *
   * @param measuredAngle The current measured angle from the encoder (in robot frame: 0° =
   *     vertical)
   * @param setpointAngle The current PID setpoint (optional - empty if not in position control)
   * @param goalAngle The goal position (optional - empty if not targeting a position)
   * @param atGoal Whether the mechanism is at the goal position
   */
  public void update(
      Rotation2d measuredAngle,
      Optional<Rotation2d> setpointAngle,
      Optional<Rotation2d> goalAngle,
      boolean atGoal) {

    // Transform from robot frame (0° = vertical) to Mechanism2d frame (0° = horizontal)
    // Transformation: mechanism_angle = 90° - robot_angle
    double measuredAngleMech2d = 90.0 - measuredAngle.getDegrees();

    // Update measured arm angle
    measuredArm.setAngle(measuredAngleMech2d);

    // Update setpoint arm (hide if no setpoint)
    if (setpointAngle.isPresent()) {
      setpointArm.setLength(armLength * 0.9);
      double setpointAngleMech2d = 90.0 - setpointAngle.get().getDegrees();
      setpointArm.setAngle(setpointAngleMech2d);
    } else {
      setpointArm.setLength(0); // Hide by setting length to 0
    }

    // Update goal arm (hide if no goal)
    if (goalAngle.isPresent()) {
      goalArm.setLength(armLength * 0.8);
      double goalAngleMech2d = 90.0 - goalAngle.get().getDegrees();
      goalArm.setAngle(goalAngleMech2d);
    } else {
      goalArm.setLength(0); // Hide by setting length to 0
    }

    // Update background color based on at-goal status
    if (atGoal) {
      mechanism.setBackgroundColor(new Color8Bit(Color.kDarkGreen));
    } else {
      mechanism.setBackgroundColor(new Color8Bit(Color.kBlack));
    }

    // Publish to SmartDashboard for Glass/AdvantageScope Mechanism2d view
    SmartDashboard.putData(name + " Visualizer", mechanism);
    Logger.recordOutput(name + "/Visualizer/Mechanism2d", mechanism);

    // Log the 3D pose for AdvantageScope 3D visualization
    // The rotation is around the Y axis (pitch) since the arm pivots up/down
    // Keep the base position fixed, only change the rotation
    Pose3d pose3d =
        new Pose3d(
            baseOffset.getTranslation(),
            new Rotation3d(
                Radians.of(baseOffset.getRotation().getX()),
                Radians.of(baseOffset.getRotation().getY())
                    .plus(Degrees.of(measuredAngle.getDegrees())),
                Radians.of(baseOffset.getRotation().getZ())));

    Logger.recordOutput(name + "/Visualizer/Pose3d", pose3d);
    Logger.recordOutput(name + "/Visualizer/MeasuredAngle_deg", measuredAngle.getDegrees());

    if (setpointAngle.isPresent()) {
      Logger.recordOutput(name + "/Visualizer/SetpointAngle_deg", setpointAngle.get().getDegrees());
    }
    if (goalAngle.isPresent()) {
      Logger.recordOutput(name + "/Visualizer/GoalAngle_deg", goalAngle.get().getDegrees());
    }
  }

  /**
   * Simplified update method when only the measured angle is available.
   *
   * @param measuredAngle The current measured angle from the encoder
   */
  public void update(Rotation2d measuredAngle) {
    update(measuredAngle, Optional.empty(), Optional.empty(), false);
  }

  /**
   * Update with measured angle and goal position.
   *
   * @param measuredAngle The current measured angle
   * @param goalAngle The goal position
   * @param atGoal Whether the mechanism is at the goal
   */
  public void update(Rotation2d measuredAngle, Rotation2d goalAngle, boolean atGoal) {
    update(measuredAngle, Optional.of(goalAngle), Optional.of(goalAngle), atGoal);
  }
}
