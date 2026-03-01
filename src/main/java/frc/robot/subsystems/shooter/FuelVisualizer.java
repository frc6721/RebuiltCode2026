package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
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
import frc.lib.fuelSim.FuelSim;
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
 * <p><b>FuelSim v1.0.0 Integration:</b>
 *
 * <p>This class delegates coordinate transforms and velocity decomposition to the FuelSim library's
 * {@code launchFuel} method, which accepts a robot-relative turret yaw and launch height so it can
 * leverage the registered robot pose supplier internally.
 */
public class FuelVisualizer {

  // Gravitational acceleration (m/s²) — used for local trajectory preview only
  private static final double GRAVITY = 9.81;

  // The shared FuelSim instance (passed in via constructor — no longer a singleton)
  private final FuelSim fuelSim;

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
   * <p>As of FuelSim v1.0.0, FuelSim is no longer a singleton. Pass the shared instance in so this
   * visualizer can call {@code launchFuel} on it.
   *
   * @param fuelSim The shared FuelSim instance created in RobotContainer
   */
  public FuelVisualizer(FuelSim fuelSim) {
    this.fuelSim = fuelSim;
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

    linearVelMps = linearVelMps * 0.75;

    return MetersPerSecond.of(linearVelMps);
  }

  /**
   * Launches a virtual fuel into the simulation using the FuelSim library's built-in launch API.
   *
   * <p>As of FuelSim v1.0.0, the library handles coordinate transforms internally using the
   * registered robot pose supplier. We just provide:
   *
   * <ul>
   *   <li>The linear launch velocity (from flywheel speed)
   *   <li>The hood angle (launch angle from horizontal)
   *   <li>The turret yaw in <b>robot-relative</b> coordinates — 180° because our shooter points
   *       backward
   *   <li>The launch height above the ground
   * </ul>
   *
   * <p>This method:
   *
   * <ol>
   *   <li>Checks if the hopper has fuel (returns early if empty)
   *   <li>Removes one fuel from the virtual hopper
   *   <li>Delegates to {@code fuelSim.launchFuel()} with robot-relative yaw
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

    // The shooter is mounted on the back of the robot, so the robot-relative yaw is 180°.
    // FuelSim v1.0.0 uses robot-relative yaw and leverages the registered pose supplier
    // internally to compute field-relative velocity — no manual rotation needed here.
    fuelSim.launchFuel(
        linearVel,
        angle,
        Degrees.of(180), // turretYaw: 180° = pointing backward in robot-relative coords
        ShooterConstants.FuelSim.HEIGHT_FROM_GROUND // launch height above the ground
        );

    // Update launch timestamp (using System.currentTimeMillis() for timing)
    lastLaunchTime = System.currentTimeMillis() / 1000.0;

    // Log the launch event for debugging
    Logger.recordOutput("Shooter/FuelSim/LaunchVelocity_mps", linearVel.in(MetersPerSecond));
    Logger.recordOutput("Shooter/FuelSim/HoodAngle_deg", angle.in(Degrees));
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

    // Get initial position and velocity by computing them inline.
    // This mirrors the same logic FuelSim uses internally for the actual launched fuel,
    // keeping the trajectory preview consistent with what will be launched.
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldSpeeds = RobotState.getInstance().getFieldRelativeVelocity();

    // Shooter height above ground from constants
    double height = ShooterConstants.FuelSim.HEIGHT_FROM_GROUND.in(Meters);
    // Shooter forward/side offsets are robot-relative; rotate them by the robot's heading
    double forwardOffset = ShooterConstants.FuelSim.FORWARD_OFFSET.in(Meters);
    double sideOffset = ShooterConstants.FuelSim.SIDE_OFFSET.in(Meters);
    Translation2d fieldOffset =
        new Translation2d(forwardOffset, sideOffset).rotateBy(robotPose.getRotation());
    Translation3d initialPos =
        new Translation3d(
            robotPose.getX() + fieldOffset.getX(), robotPose.getY() + fieldOffset.getY(), height);

    // Decompose launch speed into horizontal/vertical using hood angle
    double speedMps = linearVel.in(MetersPerSecond);
    double angleRadians = angle.in(Radians);
    double horizontalSpeed = speedMps * Math.cos(angleRadians);
    double verticalSpeed = speedMps * Math.sin(angleRadians);
    // Shooter points backward (180° robot-relative), rotate to field coords
    Translation2d robotRelativeVel = new Translation2d(-horizontalSpeed, 0);
    Translation2d fieldRelativeVel = robotRelativeVel.rotateBy(robotPose.getRotation());
    Translation3d velocity =
        new Translation3d(
            fieldRelativeVel.getX() + fieldSpeeds.vxMetersPerSecond,
            fieldRelativeVel.getY() + fieldSpeeds.vyMetersPerSecond,
            verticalSpeed);

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
