// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.VirtualHopper;
import frc.lib.fuelSim.FuelSim;
import frc.robot.subsystems.shooter.io.ShooterIO;
import frc.robot.subsystems.shooter.io.ShooterIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Shooter subsystem controls the robot's game piece launching mechanism.
 *
 * <p><b>Hardware:</b> Based on AndyMark "Launcher in a Box" (modified for 22" trench clearance).
 * Single NEO motor driving a 4" urethane flywheel 1:1, with a custom aluminum flywheel for
 * momentum.
 *
 * <p><b>How It Works:</b> The flywheel spins at high speed to store rotational energy. When a game
 * piece contacts the spinning wheel, it launches. Different speeds = different distances.
 *
 * <p><b>Software Features:</b>
 *
 * <ul>
 *   <li>Closed-loop (PID) velocity control for precise flywheel speed
 *   <li>Distance-based shooting using InterpolatingDoubleTreeMap (distance → RPM)
 *   <li>FuelSim trajectory visualization for AdvantageScope
 *   <li>SysId integration for feedforward characterization
 * </ul>
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private AngularVelocity _targetFlywheelSpeed;
  private final SysIdRoutine _sysId;

  private final FuelVisualizer _fuelSimVisualizer;

  /**
   * Creates a new Shooter subsystem.
   *
   * <p>Initializes the shooter with the provided hardware IO and stops the flywheel for safety.
   * Uses RobotState singleton for pose/velocity data (decoupled from Drive subsystem).
   *
   * @param shooterIO The hardware interface for shooter control
   * @param fuelSim The shared FuelSim instance (created once in RobotContainer)
   */
  public Shooter(ShooterIO shooterIO, FuelSim fuelSim) {
    this._shooterIO = shooterIO;
    this.stopFlywheels();

    // Initialize FuelSim visualizer for trajectory and launch simulation
    _fuelSimVisualizer = new FuelVisualizer(fuelSim);

    // Configure SysId
    _sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysId/State", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage), null, this));
  }

  /**
   * Periodic method called every 20ms. Reads sensor data, logs flywheel speed/setpoints to
   * AdvantageKit, and updates FuelSim trajectory visualization.
   *
   * <p>Velocity control happens in the hardware layer (RealShooterIO) using the motor controller's
   * built-in PID.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Shooter", _shooterInputs);

    // LOGGING
    Logger.recordOutput(
        "Shooter/FlywheelSpeed/Current_RadPerSec",
        _shooterInputs._leftFlywheelMotorVelocity.in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/FlywheelSpeed/Current_RPM", _shooterInputs._leftFlywheelMotorVelocity.in(RPM));

    Logger.recordOutput(
        "Shooter/FlywheelSpeed/Desired_RadPerSec", _targetFlywheelSpeed.in(RadiansPerSecond));
    Logger.recordOutput("Shooter/FlywheelSpeed/Desired_RPM", _targetFlywheelSpeed.in(RPM));
    Logger.recordOutput("Shooter/AtTargetSpeed", this.areFlywheelsAtTargetSpeed());

    // ==================== FUEL SIM INTEGRATION ====================
    // Update trajectory visualization every loop so driver sees real-time prediction
    LinearVelocity linearSpeed =
        _fuelSimVisualizer.convertToLinearVelocity(_shooterInputs._leftFlywheelMotorVelocity);
    _fuelSimVisualizer.updateTrajectory(linearSpeed, ShooterConstants.FuelSim.HOOD_ANGLE);

    // Check if we should launch fuel (for visualization)
    Logger.recordOutput("Shooter/FuelSim/ShouldLaunch", shouldVisualizeLaunch());

    // Actually launch fuel if conditions are met
    visualizeFuelLaunch();
  }

  /**
   * Determines if fuel should be visualized as launching.
   *
   * <p>Checks: flywheels at target speed, above threshold, hopper has fuel, and rate limit passed.
   *
   * @return true if fuel should be visualized as launching
   */
  private boolean shouldVisualizeLaunch() {
    // Check flywheel is at target speed
    boolean atTargetSpeed = areFlywheelsAtTargetSpeed();

    // Check target speed is above threshold (compare using same units)
    double targetRPM = _targetFlywheelSpeed.in(RPM);
    boolean aboveThreshold = targetRPM > ShooterConstants.FuelSim.RPM_THRESHOLD_FOR_LAUNCH.in(RPM);

    // Check hopper has fuel
    boolean hasFuel = VirtualHopper.getInstance().hasFuel();

    // Check rate limit
    boolean canLaunch = _fuelSimVisualizer.canLaunch();

    return atTargetSpeed && aboveThreshold && hasFuel && canLaunch;
  }

  /**
   * Launches virtual fuel if all conditions are met.
   *
   * <p>This uses the MEASURED flywheel speed (not target) for realistic physics. The measured speed
   * reflects actual motor performance, giving more accurate trajectory prediction.
   */
  private void visualizeFuelLaunch() {
    if (!shouldVisualizeLaunch()) {
      return;
    }

    // Use MEASURED speed for realistic physics
    LinearVelocity linearSpeed =
        _fuelSimVisualizer.convertToLinearVelocity(_shooterInputs._leftFlywheelMotorVelocity);

    // Launch with hood angle from constants
    _fuelSimVisualizer.launchFuel(linearSpeed, ShooterConstants.FuelSim.HOOD_ANGLE);
  }

  /**
   * Stops the flywheel by setting target speed to zero.
   *
   * <p>The heavy flywheel takes time to spin down due to its momentum. Always wait for the flywheel
   * to fully stop before performing maintenance or reaching near the mechanism.
   */
  public void stopFlywheels() {
    _targetFlywheelSpeed = RadiansPerSecond.of(0);
    _shooterIO.stopFlywheel();
  }

  /**
   * Sets the target flywheel speed. The motor controller's PID will automatically adjust voltage to
   * reach and maintain this speed.
   *
   * <p>The flywheel will take time to spin up. Use {@link #areFlywheelsAtTargetSpeed()} to check
   * when it's ready to shoot.
   *
   * @param speed The desired flywheel angular velocity (use RPM.of(), RadiansPerSecond.of(), etc.)
   */
  public void setFlywheelSpeed(AngularVelocity speed) {
    _targetFlywheelSpeed = speed;
    _shooterIO.setFlywheelSpeed(speed);
  }

  /**
   * Calculates the required flywheel speed for a given distance using an interpolating lookup
   * table.
   *
   * <p>Uses WPILib's InterpolatingDoubleTreeMap which linearly interpolates between characterized
   * data points stored in {@link ShooterConstants.DistanceMap#SPEED_MAP}. The result is clamped to
   * configured min/max speeds for safety.
   *
   * <p><b>To characterize:</b> Place robot at known distances, adjust RPM until shots score, and
   * add (distance, RPM) pairs to SPEED_MAP. Repeat for 5-7 distances across your shooting range.
   *
   * @param distance Distance to the target (use Meters.of() or similar)
   * @return Required flywheel speed as an AngularVelocity
   */
  public AngularVelocity getSpeedForDistance(Distance distance) {
    // Convert distance to meters for lookup table
    double distanceMeters = distance.in(Meters);

    // Get interpolated speed from lookup table (returns RPM)
    double speedRPM = ShooterConstants.DistanceMap.SPEED_MAP.get(distanceMeters);

    // Clamp to min/max speeds for safety
    double minRPM = ShooterConstants.Limits.MIN_SPEED.in(RPM);
    double maxRPM = ShooterConstants.Limits.MAX_SPEED.in(RPM);
    speedRPM = Math.max(minRPM, Math.min(maxRPM, speedRPM));

    // Log the calculation for debugging
    Logger.recordOutput("Shooter/ShotCalculator/Distance_m", distanceMeters);
    Logger.recordOutput("Shooter/ShotCalculator/CalculatedSpeed_RPM", speedRPM);

    // Convert RPM to AngularVelocity and return
    return RPM.of(speedRPM);
  }

  /**
   * Checks if the flywheel has reached its target speed within a percentage tolerance.
   *
   * <p>The tolerance is defined as a percentage in {@link ShooterConstants.Software#PID_TOLERANCE}.
   * For example, with a 2% tolerance and 3000 RPM target, returns true when speed is between
   * 2940-3060 RPM.
   *
   * <p>Check this before feeding game pieces - feeding before the flywheel is up to speed results
   * in weak, inaccurate shots.
   *
   * @return true if flywheel is at target speed (within tolerance), false otherwise
   */
  public boolean areFlywheelsAtTargetSpeed() {
    return Math.abs(
            _targetFlywheelSpeed.in(RadiansPerSecond)
                - _shooterInputs._leftFlywheelMotorVelocity.in(RadiansPerSecond))
        <= Math.abs(
            _targetFlywheelSpeed.in(RadiansPerSecond) * ShooterConstants.Software.PID_TOLERANCE);
    // the tolerance is a percent error of the target speed we are allowed
  }

  /**
   * Manually controls the flywheel with a duty cycle output, bypassing PID velocity control.
   *
   * <p><b>Warning:</b> Use only for testing motor direction, verifying wiring, or emergency manual
   * control. For normal operation, use {@link #setFlywheelSpeed(AngularVelocity)} instead.
   *
   * @param output The duty cycle output (0.0 to +1.0 recommended) for the flywheel motor
   */
  public void setFlyWheelDutyCycle(double output) {
    this._shooterIO.setFlyWheelDutyCycle(output);
  }

  public void runCharacterization(Voltage volts) {
    this._shooterIO.setFlywheelVoltage(volts);
  }

  /**
   * Returns the current flywheel velocity in rad/s for feedforward characterization. The SysId
   * routine uses this to correlate applied voltage to achieved velocity.
   *
   * @return current flywheel angular velocity (rad/s)
   */
  public double getFFCharacterizationVelocity() {
    return _shooterInputs._leftFlywheelMotorVelocity.in(RadiansPerSecond);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(_sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(_sysId.dynamic(direction));
  }

  /**
   * Updates the flywheel speed to shoot at the alliance hub. Calculates the required speed based on
   * current distance and sets the flywheel accordingly.
   *
   * <p>Call continuously (e.g., from a command's execute()) to adjust speed as the robot moves. The
   * alliance hub target is automatically flipped based on alliance color.
   */
  public void updateSpeedForHub() {
    AngularVelocity targetSpeed = ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub();
    setFlywheelSpeed(targetSpeed);
  }

  /**
   * Updates the flywheel speed to shoot at a specific point on the field.
   *
   * <p>This method calculates the required flywheel speed based on the current distance to the
   * target point and sets the flywheel to that speed. Call this method continuously to adjust speed
   * as the robot moves.
   *
   * @param target The target point to shoot at (Translation3d in field coordinates)
   */
  public void updateSpeedForTarget(Translation3d target) {
    AngularVelocity targetSpeed = ShotCalculator.getInstance().getFlywheelSpeedForTarget(target);
    setFlywheelSpeed(targetSpeed);
  }

  /**
   * Checks if the robot is within effective shooting range of the alliance hub.
   *
   * <p>This can be used by commands to decide whether to attempt a shot or provide driver feedback.
   *
   * @return true if the robot is within the characterized shooting range
   */
  public boolean isInShootingRange() {
    return ShotCalculator.getInstance().isInShootingRange();
  }
}
