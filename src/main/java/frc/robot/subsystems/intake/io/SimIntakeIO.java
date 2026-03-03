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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * Simulation implementation of IntakeIO for a linear slide + roller intake.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Linear slide:</b> Uses WPILib's {@link ElevatorSim} for realistic position-based physics
 *   <li><b>Roller:</b> Simple voltage/current simulation (instant response)
 *   <li><b>Motors:</b> Uses REV SparkMaxSim for realistic motor behavior
 * </ul>
 *
 * <p>The linear slide is modeled as an elevator: voltage goes in, and the physics sim tracks
 * position and velocity. This matches how the real intake is controlled — a PID on the roboRIO
 * outputs a voltage, and the motor moves the slide to the target position. No gravity simulation is
 * needed since the slide is horizontal.
 */
public class SimIntakeIO implements IntakeIO {

  // Simulated motors
  private final SparkMax _linearMotor;
  private final SparkMax _rollerMotor;

  // REV simulation wrappers
  private final SparkMaxSim _linearSim;
  private final SparkMaxSim _rollerSim;

  // WPILib physics simulation for the linear slide (modeled as an elevator)
  private final ElevatorSim _linearPhysicsSim;

  // Tracks the last voltage applied to the linear motor so we can feed it to the physics sim
  private double _linearAppliedVoltage = 0.0;

  // Simulated roller state
  private double _rollerDutyCycle = 0.0;
  private double _rollerSimulatedCurrent = 0.0;

  /**
   * Creates a new SimIntakeIO with ElevatorSim-based position simulation for the linear slide.
   *
   * <p>Initializes simulated motors using constants from IntakeConstants, and creates an {@link
   * ElevatorSim} to model the slide physics (voltage → position).
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

    // Create the ElevatorSim physics model for the linear slide.
    // ElevatorSim tracks position (meters) and velocity (m/s) based on applied voltage.
    // We disable gravity simulation since the slide is horizontal.
    _linearPhysicsSim =
        new ElevatorSim(
            IntakeConstants.Mechanical.LINEAR_MOTOR, // Motor type (NEO)
            IntakeConstants.Mechanical.LINEAR_GEAR_RATIO, // Gear reduction
            IntakeConstants.Mechanical.CARRIAGE_MASS_KG, // Mass of the roller assembly
            IntakeConstants.Mechanical.DRUM_RADIUS_METERS, // Drum radius (rotation → meters)
            0.0, // Min height (fully retracted)
            IntakeConstants.Mechanical.MAX_TRAVEL_METERS, // Max height (fully extended)
            false, // No gravity (horizontal slide)
            0.0); // Start fully retracted
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // --- Step the linear slide physics simulation ---
    _linearPhysicsSim.setInputVoltage(_linearAppliedVoltage);
    _linearPhysicsSim.update(0.02); // 20ms timestep

    // ElevatorSim outputs meters directly, and our encoder is now configured to read meters,
    // so no manual conversion is needed — just read the physics sim values.
    double positionMeters = _linearPhysicsSim.getPositionMeters();
    double velocityMetersPerSec = _linearPhysicsSim.getVelocityMetersPerSecond();

    // Get current draw from the physics sim
    double linearCurrent = _linearPhysicsSim.getCurrentDrawAmps();

    // Update battery simulation with combined current draw
    double totalCurrent = linearCurrent + Math.abs(_rollerSimulatedCurrent);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent));

    // Convert velocity to RPM for SparkMaxSim (it expects motor RPM)
    double motorRPM =
        velocityMetersPerSec / IntakeConstants.Mechanical.LINEAR_VELOCITY_CONVERSION_FACTOR;

    // Update SparkMax simulations so REV's sim layer stays in sync
    _linearSim.setBusVoltage(RoboRioSim.getVInVoltage());
    _linearSim.iterate(motorRPM, RoboRioSim.getVInVoltage(), 0.02);
    _rollerSim.setBusVoltage(RoboRioSim.getVInVoltage());

    // --- Populate inputs (these get logged by AdvantageKit) ---
    // Linear motor — position and velocity are already in meters / m/s
    inputs._linearMotorTemperature = Celsius.of(40.0);
    inputs._linearMotorVelocity = RadiansPerSecond.of(velocityMetersPerSec);
    inputs._linearMotorPosition = positionMeters;
    inputs._linearMotorVoltage = Volts.of(_linearAppliedVoltage);
    inputs._linearMotorCurrent = Amps.of(linearCurrent);

    // Roller motors (simple simulation — instant response, follower mirrors leader)
    inputs._rollerMotorLeaderTemperature = Celsius.of(35.0);
    inputs._rollerMotorLeaderVelocity = RadiansPerSecond.of(_rollerDutyCycle * 500.0);
    inputs._rollerMotorLeaderVoltage = Volts.of(_rollerDutyCycle * 12.0);
    _rollerSimulatedCurrent = Math.abs(_rollerDutyCycle) * 20.0;
    inputs._rollerMotorLeaderCurrent = Amps.of(_rollerSimulatedCurrent);

    inputs._rollerMotorFollowerTemperature = Celsius.of(35.0);
    inputs._rollerMotorFollowerVelocity = RadiansPerSecond.of(-_rollerDutyCycle * 500.0);
    inputs._rollerMotorFollowerVoltage = Volts.of(-_rollerDutyCycle * 12.0);
    inputs._rollerMotorFollowerCurrent = Amps.of(_rollerSimulatedCurrent);
  }

  @Override
  public void setLinearMotorVoltage(Voltage volts) {
    _linearAppliedVoltage = Math.max(-12.0, Math.min(12.0, volts.in(Volts)));
    _linearMotor.setVoltage(_linearAppliedVoltage);
  }

  @Override
  public void setLinearMotorDutyCycle(double output) {
    _linearAppliedVoltage = output * 12.0;
    _linearMotor.set(output);
  }

  @Override
  public void resetLinearEncoder() {
    // Reset the physics sim position to 0 (fully retracted, no velocity)
    _linearPhysicsSim.setState(0.0, 0.0);
  }

  @Override
  public void setRollerMotorOutput(double output) {
    _rollerDutyCycle = Math.max(-1.0, Math.min(1.0, output));
    _rollerMotor.set(_rollerDutyCycle);
  }
}
