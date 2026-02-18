package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.VirtualHopper;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The Intake subsystem controls the robot's linear slide intake mechanism.
 *
 * <p><b>Hardware:</b> A NEO motor drives a linear slide on a rail. A separate NEO motor drives
 * rollers to acquire game pieces. Position is tracked via the linear motor's internal encoder.
 *
 * <p><b>How It Works:</b> The linear motor extends the roller assembly out from the robot frame.
 * The intake starts fully retracted (position 0). Positive position = extending out. A PID
 * controller running on the roboRIO maintains the target position using the motor's internal
 * encoder.
 *
 * <p><b>Software Features:</b>
 *
 * <ul>
 *   <li>Closed-loop (PID) position control using internal encoder
 *   <li>Predefined positions (RETRACTED, EXTENDED) via enum
 *   <li>FuelSim virtual intake integration
 * </ul>
 */
public class Intake extends SubsystemBase {

  private final IntakeIO _intakeIO;
  private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();
  private IntakePosition _intakePosition;
  private final PIDController _linearPIDController;
  private final IntakeVisualizer _visualizer;

  /**
   * Enum representing predefined positions for the linear slide. Actual positions are defined in
   * {@link IntakeConstants.Positions} as LoggedNetworkNumbers (tunable from dashboard).
   */
  public enum IntakePosition {
    RETRACTED(IntakeConstants.Positions.RETRACTED),
    EXTENDED(IntakeConstants.Positions.EXTENDED);

    private final LoggedNetworkNumber _position;

    /**
     * @param position The position value for this state (from IntakeConstants)
     */
    private IntakePosition(LoggedNetworkNumber position) {
      this._position = position;
    }

    /**
     * Returns the target position for this intake state (in output rotations).
     *
     * @return The position setpoint
     */
    public double getPosition() {
      return this._position.get();
    }
  }

  /**
   * Creates a new Intake subsystem.
   *
   * <p>Initializes position to RETRACTED (0), creates a PID controller with mode-selected gains,
   * and resets the internal encoder so position 0 = current (fully retracted) position.
   *
   * @param intakeIO The hardware interface for intake control (motors and encoder)
   */
  public Intake(IntakeIO intakeIO) {
    this._intakeIO = intakeIO;

    // Assume intake starts fully retracted
    _intakePosition = IntakePosition.RETRACTED;

    // Use mode-selected PID constants (different values for sim vs real)
    _linearPIDController =
        new PIDController(
            IntakeConstants.getLinearKP(),
            IntakeConstants.getLinearKI(),
            IntakeConstants.getLinearKD());

    // Reset the internal encoder so 0 = fully retracted
    _intakeIO.resetLinearEncoder();

    // Initialize the visualizer for Mechanism2d and 3D pose output
    _visualizer = new IntakeVisualizer("Intake");
  }

  /**
   * Periodic method called every 20ms. Reads sensors, logs position data, and runs PID control loop
   * to maintain the linear slide position.
   */
  @Override
  public void periodic() {
    _intakeIO.updateInputs(_intakeInputs);
    Logger.processInputs("Intake", _intakeInputs);

    // LOGGING
    Logger.recordOutput("Intake/LinearPosition/Current", _intakeInputs._linearMotorPosition);
    Logger.recordOutput("Intake/LinearPosition/Desired", _intakePosition.getPosition());

    // Run PID control for linear slide position
    _linearPIDController.setSetpoint(_intakePosition.getPosition());
    double linearVoltage = _linearPIDController.calculate(_intakeInputs._linearMotorPosition);

    _intakeIO.setLinearMotorVoltage(linearVoltage);

    // Check if at target
    boolean atGoal =
        Math.abs(_intakeInputs._linearMotorPosition - _intakePosition.getPosition())
            < IntakeConstants.Software.POSITION_DEADBAND;

    Logger.recordOutput("Intake/Linear/AtGoal", atGoal);

    // Update the Mechanism2d and Pose3d visualization
    _visualizer.update(_intakeInputs._linearMotorPosition, _intakePosition.getPosition(), atGoal);
  }

  /**
   * Sets the intake slide to a desired position. The actual movement is handled by the PID
   * controller in {@link #periodic()}.
   *
   * @param position The desired intake position (RETRACTED or EXTENDED)
   */
  public void setIntakePosition(IntakePosition position) {
    _intakePosition = position;
  }

  /**
   * Manually controls the linear slide with a duty cycle output, bypassing PID control.
   *
   * <p><b>Warning:</b> For testing/setup only. Use {@link #setIntakePosition(IntakePosition)} for
   * normal operation.
   *
   * @param output The duty cycle output (-1.0 to +1.0) for the linear motor
   */
  public void setLinearMotorDutyCycleOutput(double output) {
    _intakeIO.setLinearMotorDutyCycle(output);
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

  /** Resets the linear encoder to zero (use when intake is known to be fully retracted). */
  public void resetEncoder() {
    _intakeIO.resetLinearEncoder();
  }

  // ==================== FUEL SIM INTEGRATION ====================

  /**
   * FuelSim callback: adds a virtual fuel to the hopper when a fuel piece enters the intake
   * bounding box, if the hopper has capacity.
   */
  public void simIntakeFuel() {
    if (VirtualHopper.getInstance().getFuelCount() < ShooterConstants.FuelSim.MAX_HOPPER_CAPACITY) {
      VirtualHopper.getInstance().addFuel();
      Logger.recordOutput("Intake/FuelSim/IntakedFuel", true);
    }
  }

  /**
   * Returns true if the intake is deployed (in EXTENDED position). Used by FuelSim to determine if
   * the intake can collect fuel.
   *
   * @return true if intake is in EXTENDED position
   */
  public boolean isDeployed() {
    return _intakePosition == IntakePosition.EXTENDED;
  }

  /**
   * Returns true if the intake linear slide is within {@link
   * IntakeConstants.Software#POSITION_DEADBAND} of its target position.
   *
   * @return true if the intake is at target
   */
  public boolean isAtTarget() {
    double currentPosition = _intakeInputs._linearMotorPosition;
    double targetPosition = _intakePosition.getPosition();
    double error = Math.abs(currentPosition - targetPosition);
    return error < IntakeConstants.Software.POSITION_DEADBAND;
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
