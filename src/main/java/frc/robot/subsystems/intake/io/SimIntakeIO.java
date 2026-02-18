package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * Simulation implementation of IntakeIO for a linear slide + roller intake.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Linear slide:</b> Simple position integration based on applied voltage
 *   <li><b>Roller:</b> Simple voltage/current simulation (instant response)
 *   <li><b>Motors:</b> Uses REV SparkMaxSim for realistic motor behavior
 * </ul>
 *
 * <p>The linear slide simulation integrates velocity to track position. No gravity or complex
 * physics needed since it's a horizontal linear rail.
 */
public class SimIntakeIO implements IntakeIO {

  // Simulated motors
  private final SparkMax _linearMotor;
  private final SparkMax _rollerMotor;

  // REV simulation wrappers
  private final SparkMaxSim _linearSim;
  private final SparkMaxSim _rollerSim;

  // Simulated linear slide state
  private double _linearPosition = 0.0; // Starts fully retracted
  private double _linearVelocity = 0.0; // Output rotations per second
  private double _linearAppliedVoltage = 0.0;

  // Simulated roller state
  private double _rollerDutyCycle = 0.0;
  private double _rollerSimulatedCurrent = 0.0;

  // Simple physics constants for the linear slide
  private static final double MAX_VELOCITY = 10.0; // Max output rotations per second
  private static final double ACCELERATION = 50.0; // Output rotations per second^2

  /**
   * Creates a new SimIntakeIO with position simulation for the linear slide.
   *
   * <p>Initializes simulated motors using constants from IntakeConstants.
   */
  public SimIntakeIO() {
    // Create simulated motors (using arbitrary CAN IDs since they don't matter in sim)
    _linearMotor = new SparkMax(50, MotorType.kBrushless);
    _rollerMotor = new SparkMax(52, MotorType.kBrushless);

    // Configure linear motor
    SparkMaxConfig linearConfig = new SparkMaxConfig();
    linearConfig
        .inverted(IntakeConstants.Hardware.LINEAR_MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.CurrentLimits.LINEAR_SMART)
        .voltageCompensation(12.0);

    linearConfig
        .encoder
        .positionConversionFactor(IntakeConstants.Mechanical.LINEAR_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(IntakeConstants.Mechanical.LINEAR_VELOCITY_CONVERSION_FACTOR);

    _linearMotor.configure(
        linearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    _linearSim = new SparkMaxSim(_linearMotor, IntakeConstants.Mechanical.LINEAR_MOTOR);
    _rollerSim = new SparkMaxSim(_rollerMotor, IntakeConstants.Mechanical.ROLLER_MOTOR);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // --- Simulate the linear slide ---
    // Calculate target velocity based on applied voltage (simple proportional model)
    double targetVelocity = (_linearAppliedVoltage / 12.0) * MAX_VELOCITY;

    // Accelerate towards target velocity
    double velocityError = targetVelocity - _linearVelocity;
    double maxDelta = ACCELERATION * 0.02; // Max change in one timestep
    if (Math.abs(velocityError) < maxDelta) {
      _linearVelocity = targetVelocity;
    } else {
      _linearVelocity += Math.signum(velocityError) * maxDelta;
    }

    // Integrate position
    _linearPosition += _linearVelocity * 0.02;

    // Clamp position to valid range (can't go below 0 = fully retracted)
    if (_linearPosition < 0.0) {
      _linearPosition = 0.0;
      _linearVelocity = Math.max(0.0, _linearVelocity);
    }

    // Approximate current draw
    double linearCurrent = Math.abs(_linearAppliedVoltage / 12.0) * 10.0;

    // Update battery simulation
    double totalCurrent = linearCurrent + Math.abs(_rollerSimulatedCurrent);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent));

    // Update SparkMax simulations
    _linearSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _linearSim.iterate(_linearVelocity * 60.0, RoboRioSim.getVInVoltage(), 0.02); // RPM
    _rollerSim.setBusVoltage(RoboRioSim.getVInVoltage());

    // --- Populate inputs ---
    // Linear motor
    inputs._linearMotorTemperature = Celsius.of(40.0);
    inputs._linearMotorVelocity = RadiansPerSecond.of(_linearVelocity * 2 * Math.PI);
    inputs._linearMotorPosition = _linearPosition;
    inputs._linearMotorVoltage = Volts.of(_linearAppliedVoltage);
    inputs._linearMotorCurrent = Amps.of(linearCurrent);

    // Roller motor (simple simulation)
    inputs._rollerMotorTemperature = Celsius.of(35.0);
    inputs._rollerMotorVelocity = RadiansPerSecond.of(_rollerDutyCycle * 500.0);
    inputs._rollerMotorVoltage = Volts.of(_rollerDutyCycle * 12.0);
    _rollerSimulatedCurrent = Math.abs(_rollerDutyCycle) * 20.0;
    inputs._rollerMotorCurrent = Amps.of(_rollerSimulatedCurrent);
  }

  @Override
  public void setLinearMotorVoltage(double volts) {
    _linearAppliedVoltage = Math.max(-12.0, Math.min(12.0, volts));
    _linearMotor.setVoltage(_linearAppliedVoltage);
  }

  @Override
  public void setLinearMotorDutyCycle(double output) {
    _linearAppliedVoltage = output * 12.0;
    _linearMotor.set(output);
  }

  @Override
  public void resetLinearEncoder() {
    _linearPosition = 0.0;
    _linearVelocity = 0.0;
  }

  @Override
  public void setRollerMotorOutput(double output) {
    _rollerDutyCycle = Math.max(-1.0, Math.min(1.0, output));
    _rollerMotor.set(_rollerDutyCycle);
  }
}
