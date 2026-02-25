package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizes the linear slide intake mechanism using Mechanism2d and logs a Pose3d for
 * AdvantageScope 3D visualization.
 *
 * <p><b>Physical geometry:</b> The intake slides outward at a fixed angle of 20° below horizontal
 * (negative pitch in the WPILib robot coordinate frame, since X = forward). Total travel is {@link
 * IntakeConstants.Visualization#MAX_TRAVEL_INCHES} inches.
 *
 * <p>The Mechanism2d canvas shows the slide as a horizontal bar that extends left-to-right, where:
 *
 * <ul>
 *   <li>0 (left) = fully retracted
 *   <li>MAX_TRAVEL (right) = fully extended
 * </ul>
 *
 * <p>Three overlapping bars are drawn:
 *
 * <ul>
 *   <li><b>Green:</b> Current measured position
 *   <li><b>Yellow:</b> Goal / target position
 *   <li><b>White lines:</b> Min/max travel bounds
 * </ul>
 *
 * <p>The Pose3d output shows the roller assembly moving along the slide axis in 3D space for
 * AdvantageScope's 3D robot viewer.
 */
public class IntakeVisualizer {

  // ---------- Mechanism2d canvas ----------
  private static final double CANVAS_WIDTH = 1.2; // meters (extra space past max travel)
  private static final double CANVAS_HEIGHT = 0.4; // meters (tall enough to see the bars)
  private static final double ROOT_X = 0.05; // root X on canvas (left edge, slightly inset)
  private static final double ROOT_Y = 0.2; // root Y on canvas (vertical center)

  // Mechanism2d components
  private final LoggedMechanism2d _mechanism;
  private final LoggedMechanismLigament2d _measuredBar;
  private final LoggedMechanismLigament2d _goalBar;
  private final LoggedMechanismLigament2d _minBound;
  private final LoggedMechanismLigament2d _maxBound;

  // ---------- Config ----------
  private final String _name;

  /**
   * Creates a new IntakeVisualizer.
   *
   * @param name The name used for logging keys (e.g., "Intake")
   */
  public IntakeVisualizer(String name) {
    _name = name;

    // Scale: map max travel (inches) to canvas width (leave room for the bound marker)
    // Use [0, maxTravelMeters] mapped onto [ROOT_X, ROOT_X + maxTravelMeters_canvas]

    _mechanism = new LoggedMechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT, new Color8Bit(Color.kBlack));

    // Root is at the retracted end (left side) of the slide
    LoggedMechanismRoot2d root = _mechanism.getRoot(name + "_Root", ROOT_X, ROOT_Y);

    // Min bound indicator (zero travel) — just a short vertical tick at root
    _minBound =
        new LoggedMechanismLigament2d(
            name + "_MinBound",
            0.02, // very short marker
            90.0, // vertical
            3,
            new Color8Bit(Color.kWhite));

    // Max bound indicator — a short tick at the maximum travel distance
    LoggedMechanismRoot2d maxRoot =
        _mechanism.getRoot(
            name + "_MaxRoot", ROOT_X + IntakeConstants.Visualization.MAX_TRAVEL.in(Meters), ROOT_Y);
    _maxBound =
        new LoggedMechanismLigament2d(
            name + "_MaxBound", 0.02, 90.0, 3, new Color8Bit(Color.kWhite));

    // Goal bar (yellow, slightly thinner — drawn first so measured draws on top)
    _goalBar =
        new LoggedMechanismLigament2d(
            name + "_Goal",
            0.0, // length updated each loop
            0.0, // horizontal (0° = right)
            4,
            new Color8Bit(Color.kYellow));

    // Measured bar (green, thicker)
    _measuredBar =
        new LoggedMechanismLigament2d(
            name + "_Measured",
            0.0, // length updated each loop
            0.0, // horizontal
            6,
            new Color8Bit(Color.kGreen));

    // Attach to roots
    root.append(_minBound);
    root.append(_goalBar);
    root.append(_measuredBar);
    maxRoot.append(_maxBound);
  }

  /**
   * Updates the visualizer with the current mechanism state.
   *
   * <p>Call this from {@link frc.robot.subsystems.intake.Intake#periodic()}.
   *
   * @param currentPosition The current linear slide position in encoder output rotations (0 =
   *     retracted)
   * @param goalPosition The goal / setpoint position in the same units
   * @param atGoal Whether the mechanism is within the deadband of its goal
   */
  public void update(double currentPosition, double goalPosition, boolean atGoal) {
    // Convert encoder rotations → meters of linear travel for display
    double currentMeters = positionToMeters(currentPosition);
    double goalMeters = positionToMeters(goalPosition);

    // Clamp to valid travel range for display
    currentMeters =
        Math.max(0.0, Math.min(currentMeters, IntakeConstants.Visualization.MAX_TRAVEL.in(Meters)));
    goalMeters =
        Math.max(0.0, Math.min(goalMeters, IntakeConstants.Visualization.MAX_TRAVEL.in(Meters)));

    // Update bar lengths (Mechanism2d length = how far the ligament extends from root)
    _measuredBar.setLength(currentMeters);
    _goalBar.setLength(goalMeters);

    // Background color: dark green when at goal, black otherwise
    _mechanism.setBackgroundColor(
        atGoal ? new Color8Bit(Color.kDarkGreen) : new Color8Bit(Color.kBlack));

    // Publish Mechanism2d to SmartDashboard and AdvantageKit
    SmartDashboard.putData(_name + " Visualizer", _mechanism);
    Logger.recordOutput(_name + "/Visualizer/Mechanism2d", _mechanism);

    // --- 3D Pose ---
    // SLIDE_ANGLE_DEGREES follows the WPILib pitch convention directly:
    //   positive pitch = nose tilts DOWN (right-hand rule around +Y axis)
    //   negative pitch = nose tilts UP
    //
    // WPILib robot frame: X = forward, Y = left, Z = up
    //
    // Decompose the travel distance along the slide axis into robot-frame X and Z:
    //   dX =  travel * cos(pitch)  — forward component (always positive for extending intake)
    //   dZ = -travel * sin(pitch)  — vertical component
    //                                 positive pitch → sin > 0 → negate → dZ < 0 (downward) ✓
    //                                 negative pitch → sin < 0 → negate → dZ > 0 (upward)   ✓
    double pitchRad = Math.toRadians(IntakeConstants.Visualization.SLIDE_ANGLE_DEGREES);
    double dx = currentMeters * Math.cos(pitchRad);  // forward along robot X
    double dz = -currentMeters * Math.sin(pitchRad); // down when pitch is positive

    Translation3d baseTranslation = IntakeConstants.Visualization.BASE_OFFSET;
    Pose3d rollerPose =
        new Pose3d(
            new Translation3d(
                baseTranslation.getX() + dx, baseTranslation.getY(), baseTranslation.getZ() + dz),
            // Fixed rotation: roller assembly sits at the slide's pitch angle (WPILib convention)
            new Rotation3d(0.0, pitchRad, 0.0));

    // rollerPose = new Pose3d();
    Logger.recordOutput(_name + "/Visualizer/Pose3d", rollerPose);
    // Logger.recordOutput(_name + "/Visualizer/Pose3d2", rollerPose);

    // Scalar logs for easy graphing
    Logger.recordOutput(_name + "/Visualizer/CurrentPosition_m", currentMeters);
    Logger.recordOutput(_name + "/Visualizer/GoalPosition_m", goalMeters);
    Logger.recordOutput(_name + "/Visualizer/AtGoal", atGoal);
  }

  /**
   * Converts encoder position (output rotations) to meters of linear travel for display.
   *
   * <p>This uses the ratio: {@link IntakeConstants.Visualization#MAX_TRAVEL} / {@link
   * IntakeConstants.Positions#EXTENDED} so that the EXTENDED setpoint maps to the full travel.
   *
   * @param encoderRotations Position in output rotations
   * @return Equivalent travel distance in meters
   */
  private double positionToMeters(double encoderRotations) {
    double extendedRotations = IntakeConstants.Positions.EXTENDED.get();
    if (extendedRotations == 0.0) return 0.0; // avoid divide-by-zero
    return (encoderRotations / extendedRotations) * IntakeConstants.Visualization.MAX_TRAVEL.in(Meters);
  }
}
