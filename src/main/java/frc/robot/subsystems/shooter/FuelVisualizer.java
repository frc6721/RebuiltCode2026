package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.VirtualHopper;
import frc.lib.feulSim.FuelSim;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Handles trajectory visualization and fuel launching for the FuelSim system.
 *
 * <p>This class is responsible for:
 *
 * <ul>
 *   <li>Calculating realistic projectile trajectories using physics (kinematics with gravity)
 *   <li>Converting flywheel RPM to linear launch velocity
 *   <li>Transforming robot-relative velocities to field-relative coordinates
 *   <li>Spawning virtual fuel in the simulation when launching
 *   <li>Logging trajectory data for visualization in AdvantageScope
 * </ul>
 *
 * <p><b>Physics Background:</b>
 *
 * <p>The trajectory calculation uses basic projectile motion equations:
 *
 * <ul>
 *   <li>x(t) = x₀ + vₓ × t (horizontal position)
 *   <li>y(t) = y₀ + vᵧ × t (lateral position)
 *   <li>z(t) = z₀ + vᵤ × t - ½ × g × t² (vertical position with gravity)
 * </ul>
 *
 * <p>Where g = 9.81 m/s² is gravitational acceleration on Earth.
 *
 * <p><b>Coordinate Transformations:</b>
 *
 * <p>The shooter velocity is initially in robot-relative coordinates (forward from the shooter). To
 * get field-relative trajectory, we:
 *
 * <ol>
 *   <li>Calculate launch velocity from flywheel speed and hood angle
 *   <li>Rotate by robot heading to convert to field coordinates
 *   <li>Add robot's field velocity (robot is moving while shooting)
 * </ol>
 *
 * <p><b>RobotState Integration:</b>
 *
 * <p>This class uses the centralized RobotState singleton to get robot pose and velocity, removing
 * the need for pose/speed suppliers to be passed in.
 */
public class FuelVisualizer {

  // Gravitational acceleration (m/s²)
  private static final double GRAVITY = 9.81;

  // Trajectory visualization array
  private final Translation3d[] trajectory;

  // Timestamp of last fuel launch (microseconds converted to seconds)
  private double lastLaunchTime = 0.0;

  /**
   * Gets the time step between trajectory points based on the total time span and number of points.
   *
   * <p>This is calculated as: timeStep = totalTimeSpan / numberOfPoints
   *
   * <p>For example, with 50 points and 2 second span: 2.0 / 50 = 0.04 seconds per point
   *
   * @return The time step in seconds between each trajectory visualization point
   */
  private double getTrajectoryTimeStep() {
    return ShooterConstants.FuelSim.TRAJECTORY_TIME_SPAN_SECONDS
        / ShooterConstants.FuelSim.TRAJECTORY_POINTS;
  }

  /**
   * Creates a new FuelVisualizer.
   *
   * <p>This class now uses RobotState.getInstance() to get robot pose and velocity, so no suppliers
   * need to be passed in.
   */
  public FuelVisualizer() {
    // Initialize trajectory array with size from constants
    this.trajectory = new Translation3d[ShooterConstants.FuelSim.TRAJECTORY_POINTS];
    for (int i = 0; i < trajectory.length; i++) {
      trajectory[i] = new Translation3d();
    }
  }

  /**
   * Converts flywheel angular velocity (RPM) to linear velocity at the wheel edge.
   *
   * <p><b>Physics Explanation:</b>
   *
   * <p>When a wheel spins, a point on its edge moves in a circle. The linear velocity of that point
   * is:
   *
   * <pre>
   * v = ω × r
   * </pre>
   *
   * <p>Where:
   *
   * <ul>
   *   <li>v = linear velocity (m/s)
   *   <li>ω = angular velocity (rad/s)
   *   <li>r = radius (m)
   * </ul>
   *
   * <p>Since we have RPM and diameter, the formula becomes:
   *
   * <pre>
   * v = (RPM / 60) × π × diameter
   *   = (RPM / 60) × circumference
   * </pre>
   *
   * <p>This is the velocity that gets imparted to the fuel when it contacts the wheel!
   *
   * @param angularVelocity The flywheel angular velocity (typically from motor encoder)
   * @return The linear velocity at the wheel edge
   */
  public LinearVelocity convertToLinearVelocity(AngularVelocity angularVelocity) {
    // Get angular velocity in radians per second
    double omegaRadPerSec = angularVelocity.in(RadiansPerSecond);

    // Calculate radius from diameter (using .in() to convert Distance to double)
    double radiusMeters = ShooterConstants.FuelSim.WHEEL_DIAMETER.in(Meters) / 2.0;

    // v = ω × r (linear velocity = angular velocity × radius)
    double linearVelMps = omegaRadPerSec * radiusMeters;

    return MetersPerSecond.of(linearVelMps);
  }

  /**
   * Calculates the initial velocity vector for launched fuel in field coordinates.
   *
   * <p><b>Physics Explanation:</b>
   *
   * <p>The fuel is launched at a fixed angle (hood angle) from horizontal. We decompose this into
   * horizontal and vertical components using trigonometry:
   *
   * <pre>
   * vₕₒᵣᵢᵤₒₙₜₐₗ = v × cos(θ)  (forward component)
   * vᵥₑᵣₜᵢcₐₗ = v × sin(θ)    (upward component)
   * </pre>
   *
   * <p>The horizontal velocity is initially in robot-relative coordinates (pointing forward from
   * the shooter). We rotate it by the robot's heading to get field coordinates.
   *
   * <p>Finally, we add the robot's velocity because the robot is moving while shooting. Think of
   * throwing a ball from a moving car - the ball inherits the car's velocity!
   *
   * @param linearVel The linear launch velocity magnitude
   * @param angle The launch angle (hood angle) from horizontal
   * @return Field-relative 3D velocity vector
   */
  public Translation3d calculateLaunchVelocity(LinearVelocity linearVel, Angle angle) {
    // Get current robot state from centralized RobotState
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldSpeeds = RobotState.getInstance().getFieldRelativeVelocity();

    // Convert units
    double speedMps = linearVel.in(MetersPerSecond);
    double angleRadians = angle.in(Radians);

    // Decompose launch velocity into horizontal and vertical components
    // Using trig: cos gives the "adjacent" side (horizontal), sin gives "opposite" (vertical)
    double horizontalSpeed = speedMps * Math.cos(angleRadians);
    double verticalSpeed = speedMps * Math.sin(angleRadians);

    // Create robot-relative horizontal velocity (pointing backward from robot,
    // since the shooter is mounted on the back of the robot)
    Translation2d robotRelativeVel = new Translation2d(-horizontalSpeed, 0);

    // Rotate by robot heading to convert to field coordinates
    // This is the key transformation - the shooter points opposite the robot's heading!
    Translation2d fieldRelativeVel = robotRelativeVel.rotateBy(robotPose.getRotation());

    // Add robot's velocity (fuel inherits robot's motion)
    double fieldVelX = fieldRelativeVel.getX() + fieldSpeeds.vxMetersPerSecond;
    double fieldVelY = fieldRelativeVel.getY() + fieldSpeeds.vyMetersPerSecond;

    // Return full 3D velocity vector
    return new Translation3d(fieldVelX, fieldVelY, verticalSpeed);
  }

  /**
   * Calculates the shooter's 3D position on the field.
   *
   * <p>The shooter has an offset from the robot center (forward and up). We need to transform this
   * robot-relative offset to field coordinates.
   *
   * @return The shooter's position in field coordinates (x, y, z in meters)
   */
  private Translation3d getShooterPosition() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();

    // Get shooter offset from constants (robot-relative) - convert Distance to meters
    double forwardOffset = ShooterConstants.FuelSim.FORWARD_OFFSET.in(Meters);
    double sideOffset = ShooterConstants.FuelSim.SIDE_OFFSET.in(Meters);
    double height = ShooterConstants.FuelSim.HEIGHT_FROM_GROUND.in(Meters);

    // Create 2D offset and rotate by robot heading
    Translation2d robotRelativeOffset = new Translation2d(forwardOffset, sideOffset);
    Translation2d fieldRelativeOffset = robotRelativeOffset.rotateBy(robotPose.getRotation());

    // Combine robot position + offset + height
    return new Translation3d(
        robotPose.getX() + fieldRelativeOffset.getX(),
        robotPose.getY() + fieldRelativeOffset.getY(),
        height);
  }

  /**
   * Launches a virtual fuel into the simulation.
   *
   * <p>This method:
   *
   * <ol>
   *   <li>Checks if the hopper has fuel (returns early if empty)
   *   <li>Removes one fuel from the virtual hopper
   *   <li>Calculates the shooter position in field coordinates
   *   <li>Calculates the launch velocity (with robot motion compensation)
   *   <li>Spawns the fuel in the FuelSim system
   *   <li>Updates the last launch timestamp
   * </ol>
   *
   * @param linearVel The linear velocity at the flywheel edge
   * @param angle The hood angle (launch angle from horizontal)
   */
  public void launchFuel(LinearVelocity linearVel, Angle angle) {
    // Check if we have fuel to launch
    if (!VirtualHopper.getInstance().hasFuel()) {
      return;
    }

    // Remove fuel from hopper
    VirtualHopper.getInstance().removeFuel();

    // Calculate position and velocity
    Translation3d position = getShooterPosition();
    Translation3d velocity = calculateLaunchVelocity(linearVel, angle);

    // Spawn fuel in simulation
    FuelSim.getInstance().spawnFuel(position, velocity);

    // Update launch timestamp (using System.currentTimeMillis() for timing)
    lastLaunchTime = System.currentTimeMillis() / 1000.0;

    // Log the launch event for debugging
    Logger.recordOutput("Shooter/FuelSim/LastLaunchPosition", position);
    Logger.recordOutput("Shooter/FuelSim/LastLaunchVelocity", velocity);
  }

  /**
   * Updates the trajectory visualization.
   *
   * <p>This calculates where the fuel would go if launched right now, creating a series of points
   * along the predicted path. This is updated every loop so the driver can see where their shot
   * will go before firing.
   *
   * <p><b>Physics - Projectile Motion:</b>
   *
   * <p>For a projectile launched with initial velocity (vₓ, vᵧ, vᵤ) from position (x₀, y₀, z₀), the
   * position at time t is:
   *
   * <pre>
   * x(t) = x₀ + vₓ × t           (horizontal - no acceleration)
   * y(t) = y₀ + vᵧ × t           (lateral - no acceleration)
   * z(t) = z₀ + vᵤ × t - ½gt²   (vertical - gravity pulls down)
   * </pre>
   *
   * <p>The -½gt² term is why the trajectory curves downward - gravity is constantly accelerating
   * the fuel toward the ground at 9.81 m/s².
   *
   * @param linearVel The current linear velocity at flywheel edge
   * @param angle The hood angle
   */
  public void updateTrajectory(LinearVelocity linearVel, Angle angle) {
    // Convert linear velocity back to angular velocity to check RPM threshold
    double linearVelMps = linearVel.in(MetersPerSecond);
    double radiusMeters = ShooterConstants.FuelSim.WHEEL_DIAMETER.in(Meters) / 2.0;
    double angularVelRadPerSec = linearVelMps / radiusMeters;
    double rpm = RadiansPerSecond.of(angularVelRadPerSec).in(RPM);

    // If flywheel speed is below threshold, clear trajectory and return
    if (rpm < ShooterConstants.FuelSim.RPM_THRESHOLD_FOR_LAUNCH.in(RPM)) {
      // Clear all trajectory points
      for (int i = 0; i < trajectory.length; i++) {
        trajectory[i] = new Translation3d();
      }
      Logger.recordOutput("Shooter/Trajectory", trajectory);
      return;
    }

    // Get initial position and velocity
    Translation3d initialPos = getShooterPosition();
    Translation3d velocity = calculateLaunchVelocity(linearVel, angle);

    // Get the time step for this trajectory calculation
    double timeStep = getTrajectoryTimeStep();

    // Calculate trajectory points
    boolean hitGround = false;
    Translation3d groundContactPoint = null;

    for (int i = 0; i < trajectory.length; i++) {
      // If we've already hit the ground, use the ground contact point for remaining points
      if (hitGround) {
        trajectory[i] = groundContactPoint;
        continue;
      }

      // Time for this point
      double t = i * timeStep;

      // Apply kinematic equations
      // x = x₀ + vₓ × t (constant velocity in x)
      double x = initialPos.getX() + velocity.getX() * t;

      // y = y₀ + vᵧ × t (constant velocity in y)
      double y = initialPos.getY() + velocity.getY() * t;

      // z = z₀ + vᵤ × t - ½ × g × t²
      // The ½gt² comes from integrating constant acceleration twice
      double z = initialPos.getZ() + velocity.getZ() * t - 0.5 * GRAVITY * t * t;

      // Check if we hit the ground
      if (z <= 0) {
        hitGround = true;
        z = 0; // Set exactly to ground level for this final point
        groundContactPoint = new Translation3d(x, y, z);
      }

      trajectory[i] = new Translation3d(x, y, z);
    }

    // Log trajectory for visualization in AdvantageScope
    Logger.recordOutput("Shooter/Trajectory", trajectory);
  }

  /**
   * Checks if enough time has passed since the last launch to fire again.
   *
   * <p>This implements a rate limit so we don't spam fuel launches. The delay is configurable via
   * SHOOTER_LAUNCH_RATE constant.
   *
   * @return true if we can launch (enough time has passed), false otherwise
   */
  public boolean canLaunch() {
    double currentTime = System.currentTimeMillis() / 1000.0;
    return (currentTime - lastLaunchTime) >= ShooterConstants.FuelSim.LAUNCH_INTERVAL.in(Seconds);
  }
}
