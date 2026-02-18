package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * Simulation implementation of IntakeIO that uses physics simulation for realistic behavior.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Pivot:</b> Uses WPILib's SingleJointedArmSim for realistic arm physics with gravity
 *   <li><b>Roller:</b> Simple voltage/current simulation without physics (instant response)
 *   <li><b>Motors:</b> Uses REV SparkMaxSim for realistic motor behavior
 * </ul>
 *
 * <p>The simulation includes:
 *
 * <ul>
 *   <li>Gravity effects on the arm
 *   <li>Moment of inertia and arm length from IntakeConstants
 *   <li>Min/max angle limits (hard stops)
 *   <li>Battery voltage simulation
 * </ul>
 */
public class SimIntakeIO implements IntakeIO {

  // Simulated motors
  private final SparkMax _rightPivotMotor;
  private final SparkMax _leftPivotMotor;
  private final SparkMax _rollerMotor;

  // REV simulation wrappers
  private final SparkMaxSim _rightPivotSim;
  private final SparkMaxSim _leftPivotSim;
  private final SparkMaxSim _rollerSim;

  // WPILib physics simulation for the pivot arm
  private final SingleJointedArmSim _pivotArmSim;

  // Simulated roller state
  private double _rollerDutyCycle = 0.0;
  private double _rollerSimulatedCurrent = 0.0;

  // Pivot control state
  private double _pivotAppliedVoltage = 0.0;

  /**
   * Creates a new SimIntakeIO with physics simulation.
   *
   * <p>Initializes simulated motors and the arm physics simulation using constants from
   * IntakeConstants.
   */
  public SimIntakeIO() {
    // Create simulated motors (using arbitrary CAN IDs since they don't matter in sim)
    _rightPivotMotor = new SparkMax(50, MotorType.kBrushless);
    _leftPivotMotor = new SparkMax(51, MotorType.kBrushless);
    _rollerMotor = new SparkMax(52, MotorType.kBrushless);

    // Configure right pivot motor
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .inverted(IntakeConstants.Hardware.RIGHT_PIVOT_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.PIVOT_SMART)
        .voltageCompensation(12.0);

    _rightPivotMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure left pivot motor to follow right
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).follow(_rightPivotMotor, true);

    _leftPivotMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure roller motor
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .inverted(IntakeConstants.Hardware.ROLLER_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.ROLLER_SMART)
        .voltageCompensation(12.0);

    _rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create REV simulation wrappers
    _rightPivotSim = new SparkMaxSim(_rightPivotMotor, IntakeConstants.Mechanical.PIVOT_MOTOR);
    _leftPivotSim = new SparkMaxSim(_leftPivotMotor, IntakeConstants.Mechanical.PIVOT_MOTOR);
    _rollerSim = new SparkMaxSim(_rollerMotor, IntakeConstants.Mechanical.ROLLER_MOTOR);

    // Create the arm physics simulation
    _pivotArmSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                IntakeConstants.Mechanical.PIVOT_MOTOR,
                IntakeConstants.Mechanical.PIVOT_MOI.in(KilogramSquareMeters),
                IntakeConstants.Mechanical.PIVOT_GEAR_RATIO),
            IntakeConstants.Mechanical.PIVOT_MOTOR,
            IntakeConstants.Mechanical.PIVOT_GEAR_RATIO,
            IntakeConstants.Mechanical.ARM_LENGTH.in(Meters),
            Math.toRadians(IntakeConstants.Mechanical.MIN_ANGLE_DEGREES),
            Math.toRadians(IntakeConstants.Mechanical.MAX_ANGLE_DEGREES),
            true, // Simulate gravity
            Math.toRadians(IntakeConstants.Mechanical.STARTING_ANGLE_DEGREES));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update the arm physics simulation
    _pivotArmSim.setInputVoltage(_pivotAppliedVoltage);
    _pivotArmSim.update(0.02);

    // Update battery simulation based on current draw
    double totalCurrent = _pivotArmSim.getCurrentDrawAmps() + Math.abs(_rollerSimulatedCurrent);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent));

    // Update the SparkMax simulation with the physics results
    _rightPivotSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _leftPivotSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _rollerSim.setBusVoltage(RoboRioSim.getVInVoltage());

    // Iterate the motor simulations
    _rightPivotSim.iterate(
        Math.toDegrees(_pivotArmSim.getVelocityRadPerSec()) / 360.0 * 60.0, // Convert to RPM
        RoboRioSim.getVInVoltage(),
        0.02);

    _leftPivotSim.iterate(
        Math.toDegrees(_pivotArmSim.getVelocityRadPerSec()) / 360.0 * 60.0,
        RoboRioSim.getVInVoltage(),
        0.02);

    // Right pivot motor inputs
    inputs._intakeRightPivotMotorTemperature = Celsius.of(40.0);
    inputs._intakeRightPivotMotorVelocity =
        RadiansPerSecond.of(_pivotArmSim.getVelocityRadPerSec());

    // Transform from physics frame to robot frame
    Rotation2d simAngle = Rotation2d.fromRadians(Math.PI / 2 - _pivotArmSim.getAngleRads());
    inputs._intakeRightPivotMotorPosition = simAngle;
    inputs._intakeRightPivotMotorVoltage = Volts.of(_pivotAppliedVoltage);
    inputs._intakeRightPivotMotorCurrent = Amps.of(_pivotArmSim.getCurrentDrawAmps() / 2.0);

    // Left pivot motor inputs (follower, similar values)
    inputs._intakeLeftPivotMotorTemperature = Celsius.of(40.0);
    inputs._intakeLeftPivotMotorVelocity = RadiansPerSecond.of(_pivotArmSim.getVelocityRadPerSec());
    inputs._intakeLeftPivotMotorPosition = simAngle;
    inputs._intakeLeftPivotMotorVoltage = Volts.of(_pivotAppliedVoltage);
    inputs._intakeLeftPivotMotorCurrent = Amps.of(_pivotArmSim.getCurrentDrawAmps() / 2.0);

    // Roller motor inputs (simple simulation)
    inputs._intakeRollerMotorTemperature = Celsius.of(35.0);
    inputs._intakeRollerMotorVelocity = RadiansPerSecond.of(_rollerDutyCycle * 500.0);
    inputs._intakeRollerMotorVoltage = Volts.of(_rollerDutyCycle * 12.0);
    _rollerSimulatedCurrent = Math.abs(_rollerDutyCycle) * 20.0;
    inputs._intakeRollerMotorCurrent = Amps.of(_rollerSimulatedCurrent);
  }

  @Override
  public void setPivotMotorVoltage(double volts) {
    _pivotAppliedVoltage = -Math.max(-12.0, Math.min(12.0, volts));
    _rightPivotMotor.setVoltage(_pivotAppliedVoltage);
  }

  @Override
  public void setIntakePivotDutyCycleOutput(double output) {
    _pivotAppliedVoltage = -output * 12.0;
    _rightPivotMotor.set(output);
  }

  @Override
  public void setRollerMotorOutput(double output) {
    _rollerDutyCycle = Math.max(-1.0, Math.min(1.0, output));
    _rollerMotor.set(_rollerDutyCycle);
  }
}
