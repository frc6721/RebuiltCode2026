package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
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
  private final ProfiledPIDController _linearPIDController;
  private final IntakeVisualizer _visualizer;

  /**
   * Controls whether the PID controller is actively driving the linear motor.
   *
   * <p>Defaults to {@code true} so the PID is always in charge unless a manual duty-cycle or
   * voltage override is explicitly requested. Setting a position via {@link
   * #setIntakePosition(IntakePosition)} will automatically re-enable the PID.
   */
  private boolean _pidEnabled = true;

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
     * Returns the target position for this intake state (in meters of linear travel).
     *
     * @return The position setpoint in meters
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

    // Use mode-selected PID constants (different values for sim vs real).
    // A TrapezoidProfile.Constraints object is passed in to limit how fast the intake
    // can move (maxVelocity) and how quickly it can ramp up to that speed (maxAcceleration).
    // This prevents the slider from slamming into the hard stops.
    _linearPIDController =
        new ProfiledPIDController(
            IntakeConstants.getLinearKP(),
            IntakeConstants.getLinearKI(),
            IntakeConstants.getLinearKD(),
            new TrapezoidProfile.Constraints(
                IntakeConstants.getLinearMaxVelocity(),
                IntakeConstants.getLinearMaxAcceleration()));

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
    Logger.recordOutput("Intake/Linear/PIDEnabled", _pidEnabled);

    // Run PID control for linear slide position only when PID mode is active.
    // Duty-cycle or voltage overrides disable the PID until a position is commanded again.
    if (_pidEnabled) {
      // setGoal() tells the ProfiledPIDController where we ultimately want to be.
      // Internally it generates a smooth trapezoidal motion profile between the current
      // position and the goal, respecting the max velocity and acceleration constraints.
      _linearPIDController.setGoal(_intakePosition.getPosition());
      double linearVoltage = _linearPIDController.calculate(_intakeInputs._linearMotorPosition);
      _intakeIO.setLinearMotorVoltage(Volts.of(linearVoltage));

      // Log the intermediate profiled setpoint so we can see the planned trajectory in
      // AdvantageScope — useful for verifying the motion profile shape while tuning.
      Logger.recordOutput(
          "Intake/LinearPosition/ProfiledSetpointPosition",
          _linearPIDController.getSetpoint().position);
      Logger.recordOutput(
          "Intake/LinearVelocity/ProfiledSetpointVelocity",
          _linearPIDController.getSetpoint().velocity);
    }

    // Check if at target
    boolean atGoal =
        Math.abs(_intakeInputs._linearMotorPosition - _intakePosition.getPosition())
            < IntakeConstants.Software.POSITION_DEADBAND;

    Logger.recordOutput("Intake/Linear/AtGoal", atGoal);

    // Update the Mechanism2d and Pose3d visualization
    _visualizer.update(_intakeInputs._linearMotorPosition, _intakePosition.getPosition(), atGoal);
  }

  /**
   * Returns the current intake position of the linear slide.
   *
   * @return The current intake position in meters
   */
  public double getIntakePosition() {
    return _intakePosition.getPosition();
  }

  /**
   * Sets the intake slide to a desired position and (re-)enables the PID controller.
   *
   * <p>Calling this method will always switch the intake back into closed-loop control, even if a
   * manual duty-cycle or voltage override was previously active.
   *
   * @param position The desired intake position (RETRACTED or EXTENDED)
   */
  public void setIntakePosition(IntakePosition position) {
    _intakePosition = position;
    // Always re-enable closed-loop control when a position is commanded
    _pidEnabled = true;
  }

  /**
   * Manually controls the linear slide with a duty cycle output, bypassing PID control.
   *
   * <p>This disables the PID controller so it does not fight the manual output. Call {@link
   * #setIntakePosition(IntakePosition)} or {@link #enablePID()} to return to closed-loop control.
   *
   * <p><b>Warning:</b> For testing/setup only. Use {@link #setIntakePosition(IntakePosition)} for
   * normal operation.
   *
   * @param output The duty cycle output (-1.0 to +1.0) for the linear motor
   */
  public void setLinearMotorDutyCycleOutput(double output) {
    _pidEnabled = false;
    _intakeIO.setLinearMotorDutyCycle(output);
  }

  public void homeIntake() {
    // while (_intakeIO._linearMotorCurrent < 10) {

    // }
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

  public void setRollerVoltage(Voltage voltage) {
    _intakeIO.setRollerVoltage(voltage);
  }

  /**
   * Directly applies a voltage to the linear motor, bypassing PID control.
   *
   * <p>This disables the PID controller so it does not fight the manual output. Call {@link
   * #setIntakePosition(IntakePosition)} or {@link #enablePID()} to return to closed-loop control.
   *
   * @param voltage The voltage to apply to the linear motor
   */
  public void setLinearMotorVoltage(Voltage voltage) {
    _pidEnabled = false;
    _intakeIO.setLinearMotorVoltage(voltage);
  }

  // ==================== PID CONTROL ======================

  /**
   * Returns {@code true} if the PID controller is currently active and driving the linear motor.
   *
   * @return {@code true} when closed-loop position control is enabled
   */
  public boolean isPIDEnabled() {
    return _pidEnabled;
  }

  /**
   * Enables the PID controller, returning the intake to closed-loop position control.
   *
   * <p>Use this to re-engage automatic control after a manual duty-cycle or voltage override. The
   * controller will immediately begin driving toward the last commanded {@link IntakePosition}.
   */
  public void enablePID() {
    _pidEnabled = true;
  }

  /**
   * Disables the PID controller, allowing manual control of the linear motor via duty cycle or
   * voltage methods. The PID will remain disabled until {@link #enablePID()} or a position command
   * is given.
   *
   * <p><b>Warning:</b> Use this for testing/setup only. Use {@link
   * #setIntakePosition(IntakePosition)} for normal operation to ensure the PID controller is active
   * and maintaining the desired position.
   */
  public void disablePID() {
    _pidEnabled = false;
  }

  // ==================== FUEL SIM INTEGRATION =============`=======

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
