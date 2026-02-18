package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.VirtualHopper;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The Intake subsystem controls the robot's slap-down game piece intake mechanism.
 *
 * <p><b>Hardware:</b> Two NEO motors for pivot control (leader-follower), AM Sport Gearboxes (16:1)
 * for rollers, REV Through Bore Encoder for absolute position, and ~25 Thrifty Squish Wheels on
 * belt-driven roller shafts.
 *
 * <p><b>How It Works:</b> The intake pivots down to PICKUP position, roller shafts spin to pull in
 * game pieces, then pivots back up to STOW. A PID controller with gravity feedforward (cosine of
 * angle) maintains the pivot position.
 *
 * <p><b>Software Features:</b>
 *
 * <ul>
 *   <li>Closed-loop (PID) position control with gravity feedforward
 *   <li>Predefined positions (STOW, PICKUP) via enum
 *   <li>Mechanism2d visualization for AdvantageScope
 *   <li>FuelSim virtual intake integration
 * </ul>
 */
public class Intake extends SubsystemBase {

  private final IntakeIO _intakeIO;
  private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();
  private IntakePosition _intakePosition;
  private PIDController _pivotPIDController;
  private final IntakeVisualizer _visualizer;

  /**
   * Enum representing predefined positions for the intake pivot. Actual angles are defined in
   * {@link IntakeConstants.Positions} as LoggedNetworkNumbers (tunable from dashboard).
   */
  public enum IntakePosition {
    STOW(IntakeConstants.Positions.STOW),
    PICKUP(IntakeConstants.Positions.PICKUP);

    private final LoggedNetworkNumber _angle;
    /**
     * @param angle The angle for this position (from IntakeConstants)
     */
    private IntakePosition(LoggedNetworkNumber angle) {
      this._angle = angle;
    }

    /**
     * Returns the angle for this intake position as a Rotation2d.
     *
     * @return The angle for this position
     */
    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(this._angle.get());
    }
  }

  /**
   * Creates a new Intake subsystem.
   *
   * <p>Initializes position to STOW, creates a PID controller with mode-selected gains (different
   * for sim vs real), and sets up the Mechanism2d visualizer.
   *
   * @param intakeIO The hardware interface for intake control (motors and sensors)
   */
  public Intake(IntakeIO intakeIO) {
    this._intakeIO = intakeIO;

    // assume that the intake is all the way up when first turned on
    _intakePosition = IntakePosition.STOW;

    // Use mode-selected PID constants (different values for sim vs real)
    _pivotPIDController =
        new PIDController(
            IntakeConstants.getPivotKP(),
            IntakeConstants.getPivotKI(),
            IntakeConstants.getPivotKD());

    // Initialize the visualizer for Mechanism2d and 3D pose output
    _visualizer = new IntakeVisualizer("Intake");
  }

  /**
   * Periodic method called every 20ms. Reads sensors, logs position data, runs PID control loop
   * with gravity feedforward (cosine of angle), and updates the Mechanism2d visualization.
   *
   * <p>Gravity feedforward uses cosine because torque needed is maximum at horizontal (cos(0)=1)
   * and zero at vertical (cos(90)=0).
   */
  @Override
  public void periodic() {
    _intakeIO.updateInputs(_intakeInputs);
    Logger.processInputs("Intake", _intakeInputs);

    // LOGGING
    Logger.recordOutput(
        "Intake/PivotAngle/Current_deg", _intakeInputs._intakeRightPivotMotorPosition.getDegrees());
    Logger.recordOutput("Intake/PivotAngle/Desired_deg", _intakePosition.getAngle().getDegrees());
    Logger.recordOutput(
        "Intake/PivotAngle/RawCurrent_deg",
        _intakeInputs._intakeRightPivotMotorPosition.plus(
            IntakeConstants.Hardware.PIVOT_ZERO_ROTATION));
    Logger.recordOutput(
        "Intake/PivotAngle/RawDesired_deg",
        _intakePosition.getAngle().plus(IntakeConstants.Hardware.PIVOT_ZERO_ROTATION));

    // Run PID control
    _pivotPIDController.setSetpoint(_intakePosition.getAngle().getDegrees());
    double pivotVoltage =
        _pivotPIDController.calculate(_intakeInputs._intakeRightPivotMotorPosition.getDegrees());

    // Apply feedforward for gravity compensation (uses mode-selected constant)
    double feedforward =
        IntakeConstants.getPivotFeedforward()
            * Math.cos(Math.toRadians(_intakePosition.getAngle().getDegrees()));

    _intakeIO.setPivotMotorVoltage(pivotVoltage + feedforward);

    // Update visualization with current state
    boolean atGoal =
        Math.abs(
                _intakeInputs._intakeRightPivotMotorPosition.getDegrees()
                    - _intakePosition.getAngle().getDegrees())
            < IntakeConstants.Software.PIVOT_DEADBAND;

    Logger.recordOutput("Intake/Pivot/AtGoal", atGoal);

    _visualizer.update(
        _intakeInputs._intakeRightPivotMotorPosition, _intakePosition.getAngle(), atGoal);
  }

  /**
   * Sets the intake pivot to a desired position. The actual movement is handled by the PID
   * controller in {@link #periodic()}.
   *
   * @param position The desired intake position (STOW or PICKUP)
   */
  public void setIntakePosition(IntakePosition position) {
    _intakePosition = position;
  }

  /**
   * Manually controls the intake pivot with a duty cycle output, bypassing PID control.
   *
   * <p><b>Warning:</b> For testing/setup only. Use {@link #setIntakePosition(IntakePosition)} for
   * normal operation.
   *
   * @param output The duty cycle output (-1.0 to +1.0) for the pivot motor
   */
  public void setIntakePivotDutyCycleOutput(double output) {
    _intakeIO.setIntakePivotDutyCycleOutput(output);
  }

  /**
   * Turns on the intake roller motors at {@link IntakeConstants.Roller#ACQUIRE_SPEED} to pull in
   * game pieces. Remember to call {@link #stopRollers()} when done.
   */
  public void turnOnIntakeRollers() {
    _intakeIO.setRollerMotorOutput(IntakeConstants.Roller.ACQUIRE_SPEED.get());
  }

  /** Stops the intake roller motors. */
  public void stopRollers() {
    _intakeIO.setRollerMotorOutput(0.0);
  }

  // ==================== FUEL SIM INTEGRATION ====================

  /**
   * FuelSim callback: adds a virtual fuel to the hopper when a fuel piece enters the intake
   * bounding box, if the hopper has capacity.
   */
  public void simIntakeFuel() {
    // Check if we have room in the hopper
    if (VirtualHopper.getInstance().getFuelCount() < ShooterConstants.FuelSim.MAX_HOPPER_CAPACITY) {
      VirtualHopper.getInstance().addFuel();
      Logger.recordOutput("Intake/FuelSim/IntakedFuel", true);
    }
  }

  /**
   * Returns true if the intake is deployed (in PICKUP position). Used by FuelSim to determine if
   * the intake can collect fuel.
   *
   * @return true if intake is in PICKUP position
   */
  public boolean isDeployed() {
    return _intakePosition == IntakePosition.PICKUP;
  }

  /**
   * Returns true if the intake pivot is within {@link IntakeConstants.Software#PIVOT_DEADBAND} of
   * its target position.
   *
   * @return true if the intake pivot is at target
   */
  public boolean isAtTarget() {
    double currentAngle = _intakeInputs._intakeRightPivotMotorPosition.getDegrees();
    double targetAngle = _intakePosition.getAngle().getDegrees();
    double error = Math.abs(currentAngle - targetAngle);
    return error < IntakeConstants.Software.PIVOT_DEADBAND;
  }

  /**
   * Returns true if the intake can currently collect fuel: deployed, at target position, and hopper
   * has capacity. Used as the BooleanSupplier for FuelSim's registerIntake().
   *
   * @return true if all conditions for fuel intake are met
   */
  public boolean canIntakeFuel() {
    boolean deployed = isDeployed();
    boolean atTarget = isAtTarget();
    boolean hasCapacity =
        VirtualHopper.getInstance().getFuelCount() < ShooterConstants.FuelSim.MAX_HOPPER_CAPACITY;

    // Log for debugging
    Logger.recordOutput("Intake/FuelSim/IsDeployed", deployed);
    Logger.recordOutput("Intake/FuelSim/IsAtTarget", atTarget);
    Logger.recordOutput("Intake/FuelSim/HasCapacity", hasCapacity);
    Logger.recordOutput("Intake/FuelSim/CanIntake", deployed && atTarget && hasCapacity);

    return deployed && atTarget && hasCapacity;
  }
}
