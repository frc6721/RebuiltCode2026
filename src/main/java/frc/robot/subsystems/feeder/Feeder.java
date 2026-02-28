package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.feeder.io.FeederIO;
import frc.robot.subsystems.feeder.io.FeederIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Feeder subsystem controls the mechanism that feeds game pieces from the hopper into the
 * shooter.
 *
 * <p><b>Hardware:</b> Two NEO motors (leader-follower) with AM Sport Gearbox (4:1), 4" Thrifty
 * Squish Wheels, belt-driven (36T pulleys, 120T 5mm pitch belt), with polycarbonate arc guide.
 *
 * <p><b>Control Modes:</b>
 *
 * <ul>
 *   <li><b>Duty Cycle:</b> Simple open-loop control for manual testing ({@link
 *       #setFeederSpeed(double)})
 *   <li><b>Velocity (PID + FF):</b> Closed-loop velocity control with Motion Magic acceleration
 *       profiling ({@link #setFeederVelocity(AngularVelocity)})
 *   <li><b>Voltage:</b> Direct voltage control for feedforward characterization ({@link
 *       #runCharacterization(Voltage)})
 * </ul>
 *
 * <p>Uses the AdvantageKit IO layer pattern for hardware abstraction.
 */
public class Feeder extends SubsystemBase {

  private final FeederIO _feederIO;
  private final FeederIOInputsAutoLogged _feederInputs = new FeederIOInputsAutoLogged();

  /** Target speed for closed-loop velocity control. Tracked for logging and at-speed checks. */
  private AngularVelocity _targetFeederSpeed;

  /** WPILib SysId routine for automated feedforward characterization. */
  private final SysIdRoutine _sysId;

  /**
   * Creates a new Feeder subsystem.
   *
   * <p>Initializes the feeder with the provided hardware IO, stops the motors for safety, and
   * configures the SysId routine for feedforward characterization.
   *
   * @param feederIO The hardware interface for feeder control
   */
  public Feeder(FeederIO feederIO) {
    this._feederIO = feederIO;
    this._targetFeederSpeed = RadiansPerSecond.of(0);
    this.stop();

    // Configure SysId routine for automated feedforward characterization
    // This lets WPILib's SysId tool automatically find kS and kV for the feeder
    _sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Feeder/SysId/State", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage), null, this));
  }

  /**
   * Periodic method called every 20ms. Reads sensor data, logs feeder speed/setpoints to
   * AdvantageKit.
   *
   * <p>Velocity control happens in the hardware layer (RealFeederIO) using the motor controller's
   * built-in PID + Motion Magic.
   */
  @Override
  public void periodic() {
    // Update and log sensor inputs from hardware
    _feederIO.updateInputs(_feederInputs);
    Logger.processInputs("Feeder", _feederInputs);

    // Log current and target speed for tuning and debugging
    Logger.recordOutput(
        "Feeder/FeederSpeed/Current_RadPerSec",
        _feederInputs._leftFeederMotorVelocity.in(RadiansPerSecond));
    Logger.recordOutput(
        "Feeder/FeederSpeed/Current_RPM",
        _feederInputs._leftFeederMotorVelocity.in(RevolutionsPerSecond) * 60);

    Logger.recordOutput(
        "Feeder/FeederSpeed/Desired_RadPerSec", _targetFeederSpeed.in(RadiansPerSecond));
    Logger.recordOutput(
        "Feeder/FeederSpeed/Desired_RPM", _targetFeederSpeed.in(RevolutionsPerSecond) * 60);
    Logger.recordOutput("Feeder/AtTargetSpeed", this.areFeederWheelsAtTargetSpeed());
  }

  // ==================== DUTY CYCLE CONTROL ====================

  // ==================== VELOCITY CONTROL ====================

  /**
   * Sets the target feeder velocity using closed-loop PID control with feedforward. The motor
   * controller's PID + Motion Magic will automatically adjust voltage to reach and maintain this
   * speed with smooth acceleration.
   *
   * <p>Use {@link #areFeederWheelsAtTargetSpeed()} to check when the feeder is at the target.
   *
   * @param speed The desired feeder angular velocity (use RPM.of(), RadiansPerSecond.of(), etc.)
   */
  public void setFeederVelocity(AngularVelocity speed) {
    _targetFeederSpeed = speed;
    _feederIO.setFeederVelocity(speed);
  }

  /**
   * Checks if the feeder wheels have reached their target speed within a percentage tolerance.
   *
   * <p>The tolerance is defined in {@link FeederConstants.Software#PID_TOLERANCE}. For example,
   * with a 5% tolerance and 500 RPM target, returns true when speed is between 475-525 RPM.
   *
   * @return true if feeder is at target speed (within tolerance), false otherwise
   */
  public boolean areFeederWheelsAtTargetSpeed() {
    return Math.abs(
            _targetFeederSpeed.in(RadiansPerSecond)
                - _feederInputs._leftFeederMotorVelocity.in(RadiansPerSecond))
        <= Math.abs(
            _targetFeederSpeed.in(RadiansPerSecond) * FeederConstants.Software.PID_TOLERANCE);
  }

  // ==================== CHARACTERIZATION ====================

  /**
   * Applies a voltage directly to the feeder motor, bypassing PID. Used by SysId and the manual
   * feedforward characterization command to correlate voltage → velocity.
   *
   * @param volts The voltage to apply
   */
  public void runCharacterization(Voltage volts) {
    _feederIO.setMotorVoltage(volts);
  }

  /**
   * Runs the feeder at a specific voltage. Convenience method for commands.
   *
   * @param voltage The voltage to apply to the feeder motor
   */
  public void runFeederAtVoltage(Voltage voltage) {
    _feederIO.setMotorVoltage(voltage);
  }

  /**
   * Returns the current feeder velocity in rad/s for feedforward characterization. The SysId
   * routine and manual characterization command use this to correlate applied voltage to achieved
   * velocity.
   *
   * @return current feeder angular velocity (rad/s)
   */
  public double getFFCharacterizationVelocity() {
    return _feederInputs._leftFeederMotorVelocity.in(RadiansPerSecond);
  }

  // ==================== SYSID COMMANDS ====================

  /**
   * Creates a SysId quasistatic command (slowly ramps voltage). Run forward and reverse to
   * characterize kS and kV.
   *
   * @param direction Forward or Reverse
   * @return The SysId quasistatic command
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(_sysId.quasistatic(direction));
  }

  /**
   * Creates a SysId dynamic command (applies a step voltage). Run forward and reverse to
   * characterize kA (acceleration constant).
   *
   * @param direction Forward or Reverse
   * @return The SysId dynamic command
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(_sysId.dynamic(direction));
  }

  // ==================== STOP ====================

  /**
   * Stops the feeder motor and resets the target speed to zero. Also resets the PID integral
   * accumulator to prevent windup on next start.
   */
  public void stop() {
    _targetFeederSpeed = RadiansPerSecond.of(0);
    _feederIO.setMotorVoltage(Volts.of(0.0));
  }
}
