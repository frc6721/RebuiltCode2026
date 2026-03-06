package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Constants for the Intake subsystem.
 *
 * <p>The intake is a linear slide mechanism with a roller motor. The linear motor extends/retracts
 * the roller assembly on a rail. Position is tracked using the linear motor's internal encoder
 * (starts at 0 = fully retracted, positive = extending out).
 */
public class IntakeConstants {

  /** Hardware configuration for motor inversions. */
  public static class Hardware {
    /** Linear slide motor inversion */
    public static final boolean LINEAR_MOTOR_INVERTED = true;

    /** Roller motor inversion */
    public static final boolean ROLLER_INVERTED = false;
  }

  /**
   * Physical/mechanical properties of the intake rack-and-pinion linear slide.
   *
   * <p><b>How it works:</b> A NEO motor spins through a gearbox, then drives a pinion gear that
   * meshes with a rack (a flat gear strip). As the pinion rotates, the rack (and the roller
   * assembly bolted to it) moves linearly.
   *
   * <p><b>Unit strategy:</b> All constants below define the real physical measurements. The encoder
   * conversion factors and simulation drum radius are <em>derived</em> from these, so you only need
   * to update one place when the hardware changes.
   *
   * <pre>
   *   [NEO Motor] → [Gearbox (LINEAR_GEAR_RATIO : 1)] → [Pinion (PINION_DIAMETER)] → [Rack]
   *                                                          ↕
   *                                               linear travel (meters)
   * </pre>
   */
  public static class Mechanical {

    // ── Physical hardware parameters (single source of truth) ──

    /**
     * Gear ratio between the motor and the pinion gear. A ratio of 4`:1 means the motor spins 4
     * times for every 1 rotation of the pinion.
     */
    public static final double LINEAR_GEAR_RATIO = 4.0;

    /**
     * Pitch diameter of the pinion gear in meters. This is the effective diameter where the pinion
     * meshes with the rack. For a 1-inch pitch diameter pinion: {@code Inches.of(1.0).in(Meters)}.
     *
     * <p><b>How to measure:</b> Count the teeth on the pinion and multiply by the tooth pitch (the
     * distance between teeth on the rack). Or look up the pitch diameter on the gear spec sheet.
     */
    public static final double PINION_DIAMETER_METERS = Inches.of(1.5).in(Meters);

    /**
     * Maximum linear travel of the intake slide in meters. This is the physical distance from fully
     * retracted to fully extended. Measured as 11 inches on our robot.
     */
    public static final double MAX_TRAVEL_METERS = Inches.of(11).in(Meters);

    /**
     * Mass of the intake carriage (roller assembly + any attached hardware) in kilograms. Used by
     * ElevatorSim in simulation to model inertia. Weigh the moving parts on a scale to get this
     * value.
     */
    public static final double CARRIAGE_MASS_KG = 5.0;

    /** Motor type for the linear slide (1x NEO). */
    public static final DCMotor LINEAR_MOTOR = DCMotor.getNEO(1);

    /** Roller gear ratio */
    public static final double ROLLER_GEAR_RATIO = 1.0;

    /** Motor type for the roller (1x NEO). */
    public static final DCMotor ROLLER_MOTOR = DCMotor.getNEO(1);

    // ── Derived constants (computed from physical parameters above) ──

    /**
     * Linear distance traveled per single rotation of the pinion, in meters.
     *
     * <p>For a rack-and-pinion, one full pinion rotation moves the rack by the pinion's
     * circumference: {@code π × diameter}.
     */
    public static final double METERS_PER_PINION_ROTATION = Math.PI * PINION_DIAMETER_METERS;

    /**
     * Linear distance traveled per single rotation of the motor shaft, in meters. This accounts for
     * the gearbox reduction: {@code metersPerPinionRotation / gearRatio}.
     *
     * <p>This is the key conversion factor that both the real encoder and the simulation use.
     */
    public static final double METERS_PER_MOTOR_ROTATION =
        METERS_PER_PINION_ROTATION / LINEAR_GEAR_RATIO;

    /**
     * SparkMax encoder position conversion factor. Converts raw motor rotations into meters of
     * linear travel. Set this on the SparkMax encoder so {@code encoder.getPosition()} returns
     * meters directly.
     *
     * <p>Same as {@link #METERS_PER_MOTOR_ROTATION} — named separately for clarity when configuring
     * the SparkMax.
     */
    public static final double LINEAR_POSITION_CONVERSION_FACTOR = METERS_PER_MOTOR_ROTATION;

    /**
     * SparkMax encoder velocity conversion factor. Converts motor RPM into meters per second.
     *
     * <p>{@code metersPerMotorRotation / 60} because the SparkMax returns velocity in RPM.
     */
    public static final double LINEAR_VELOCITY_CONVERSION_FACTOR = METERS_PER_MOTOR_ROTATION / 60.0;

    /**
     * Effective drum radius for WPILib's ElevatorSim, in meters. ElevatorSim models the mechanism
     * as a cable wound around a drum: {@code distance = rotations × 2π × drumRadius}.
     *
     * <p>We derive this from the pinion circumference so ElevatorSim matches the real hardware
     * exactly: {@code drumRadius = pinionCircumference / (2π) = diameter / 2}.
     */
    public static final double DRUM_RADIUS_METERS = PINION_DIAMETER_METERS / 2.0;

    // ── Visualization / mounting ──

    /**
     * Translation3d of the slide's retracted (zero) position in the robot frame. Adjust X/Y/Z to
     * match where the intake rail starts on your robot. X = forward, Y = left, Z = up (all in
     * meters).
     */
    public static final Translation3d BASE_OFFSET = new Translation3d(0.0, 0.0, 0.23);

    /**
     * Fixed angle of the slide rail, following the WPILib pitch convention.
     *
     * <p>Positive = nose tilts DOWN (right-hand rule around +Y axis, same as {@code Rotation3d}
     * pitch). Negative = nose tilts UP. Supply the physical downward angle of your intake rail as a
     * positive number.
     */
    public static final double SLIDE_ANGLE_DEGREES = 5;
  }

  /**
   * Position setpoints for the linear slide, in meters.
   *
   * <p>Position 0 = fully retracted (in). Positive values = extending out. Units are in meters of
   * linear travel (matching the encoder output after conversion). Tune these to match your
   * hardware. The maximum physical travel is {@link Mechanical#MAX_TRAVEL_METERS}.
   */
  public static class Positions {
    /** Fully retracted (stowed) position in meters. */
    public static final LoggedNetworkNumber RETRACTED =
        new LoggedNetworkNumber("Intake/Position/Retracted", 0.0);

    /** Fully extended (deployed) position in meters. */
    // public static final LoggedNetworkNumber EXTENDED =
    //     new LoggedNetworkNumber("Intake/Position/Extended", Mechanical.MAX_TRAVEL_METERS);

    /** DELETE BELOW LINE AND REVERT EXTENDED VALUE TO ABOVE. JUST FOR PID TUNING TESTING */
    public static final LoggedNetworkNumber EXTENDED =
        new LoggedNetworkNumber("Intake/Position/Extended", Inches.of(11.0).in(Meters));

    /**
     * Jostle-extend position in meters. The intake moves here during the outward half of a jostle
     * cycle to shake fuel loose in the hopper. Tunable from the dashboard.
     */
    public static final LoggedNetworkNumber JOSTLE_EXTENDED =
        new LoggedNetworkNumber("Intake/Position/JostleExtended", Inches.of(10.0).in(Meters));

    /**
     * Jostle-retract position in meters. The intake moves here during the inward half of a jostle
     * cycle. Tunable from the dashboard.
     */
    public static final LoggedNetworkNumber JOSTLE_RETRACTED =
        new LoggedNetworkNumber("Intake/Position/JostleRetracted", Inches.of(4.0).in(Meters));
  }

  /** PID tuning constants for the linear slide position control. */
  public static class PID {
    /** Real robot PID values - tuned for actual hardware */
    public static class Real {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/kP", 33.0);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/kD", 3.0);

      /**
       * Maximum velocity of the linear slide in meters per second.
       *
       * <p>The ProfiledPIDController will never command motion faster than this, even if the
       * position error is large. Tune this to keep the intake from slamming into the hard stops.
       */
      public static final LoggedNetworkNumber MAX_VELOCITY_MPS =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/MaxVelocity_mps", 1);

      /**
       * Maximum acceleration of the linear slide in meters per second squared.
       *
       * <p>Controls how quickly the slide ramps up to its maximum velocity. Lower values produce a
       * smoother, gentler motion; higher values produce faster but jerkier starts and stops.
       */
      public static final LoggedNetworkNumber MAX_ACCELERATION_MPSS =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/MaxAcceleration_mpss", 2);
    }

    /** Simulation PID values - tuned for physics simulation */
    public static class Sim {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/kP", 20);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/kD", 0.01);

      /**
       * Maximum velocity of the linear slide in meters per second (simulation).
       *
       * <p>See {@link Real#MAX_VELOCITY_MPS} for a full description.
       */
      public static final LoggedNetworkNumber MAX_VELOCITY_MPS =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/MaxVelocity_mps", 0.4);

      /**
       * Maximum acceleration of the linear slide in meters per second squared (simulation).
       *
       * <p>See {@link Real#MAX_ACCELERATION_MPSS} for a full description.
       */
      public static final LoggedNetworkNumber MAX_ACCELERATION_MPSS =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/MaxAcceleration_mpss", 0.8);
    }
  }

  /** Returns the appropriate PID kP based on current mode */
  public static double getLinearKP() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KP.get() : PID.Real.KP.get();
  }

  /** Returns the appropriate PID kI based on current mode */
  public static double getLinearKI() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KI.get() : PID.Real.KI.get();
  }

  /** Returns the appropriate PID kD based on current mode */
  public static double getLinearKD() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KD.get() : PID.Real.KD.get();
  }

  /** Returns the appropriate max velocity constraint (m/s) based on current mode */
  public static double getLinearMaxVelocity() {
    return Constants.currentMode == Constants.Mode.SIM
        ? PID.Sim.MAX_VELOCITY_MPS.get()
        : PID.Real.MAX_VELOCITY_MPS.get();
  }

  /** Returns the appropriate max acceleration constraint (m/s²) based on current mode */
  public static double getLinearMaxAcceleration() {
    return Constants.currentMode == Constants.Mode.SIM
        ? PID.Sim.MAX_ACCELERATION_MPSS.get()
        : PID.Real.MAX_ACCELERATION_MPSS.get();
  }

  /** Current limits for motor protection. */
  public static class CurrentLimits {
    public static final int LINEAR_SMART = 40;
    public static final double LINEAR_SECONDARY = 55;
    public static final int ROLLER_SMART = 50;
    public static final double ROLLER_SECONDARY = 60;
  }

  /** Roller motor settings. */
  public static class Roller {
    /** Voltage to run rollers when acquiring game pieces (negative = inward on our robot). */
    public static final LoggedNetworkNumber ACQUIRE_SPEED =
        new LoggedNetworkNumber("Intake/Roller/Acquire Speed", -4);

    /** Voltage to run rollers slowly when acquiring game pieces. */
    public static final LoggedNetworkNumber SLOW_ACQUIRE_SPEED =
        new LoggedNetworkNumber("Intake/Roller/Slow Acquire Speed", -2);

    /** Voltage to run rollers when spitting game pieces out (positive = outward). */
    public static final LoggedNetworkNumber SPIT_SPEED =
        new LoggedNetworkNumber("Intake/Roller/Spit Speed", 4);

    /** Voltage to run rollers slowly when spitting game pieces out. */
    public static final LoggedNetworkNumber SLOW_SPIT_SPEED =
        new LoggedNetworkNumber("Intake/Roller/Slow Spit Speed", 2);

    public static final LoggedNetworkNumber CURRENT_CUTOFF =
        new LoggedNetworkNumber("Intake/Roller/Current Cutoff", 40);
  }

  /**
   * Duration of one half of an intake jostle cycle (extend or retract), in seconds. Tunable from
   * the dashboard.
   */
  public static final LoggedNetworkNumber JOSTLE_HALF_CYCLE_DURATION_SECONDS =
      new LoggedNetworkNumber("Intake/Jostle/Half Cycle Duration", 0.15);

  /** Software tuning settings. */
  public static class Software {
    /** Deadband for considering linear slide "at position" (in meters) */
    public static final double POSITION_DEADBAND = 0.005; // 5mm
  }

  /** FuelSim bounding box constants for intake pickup simulation. */
  public static class FuelSim {
    /**
     * Width of the intake bounding box (robot Y axis — side to side). Centered on the robot's
     * centerline.
     */
    public static final Distance WIDTH = Inches.of(10.0);

    /**
     * Length of the intake bounding box (robot X axis — front to back). The box extends forward
     * from the front edge of the robot by this amount.
     */
    public static final Distance LENGTH = Inches.of(20.0);
  }

  // Logging
  static {
    // Rack-and-pinion physical parameters
    Logger.recordOutput("Constants/Intake/GearRatio/Linear", Mechanical.LINEAR_GEAR_RATIO);
    Logger.recordOutput("Constants/Intake/GearRatio/Roller", Mechanical.ROLLER_GEAR_RATIO);
    Logger.recordOutput("Constants/Intake/PinionDiameter_m", Mechanical.PINION_DIAMETER_METERS);
    Logger.recordOutput(
        "Constants/Intake/MetersPerPinionRotation_m", Mechanical.METERS_PER_PINION_ROTATION);
    Logger.recordOutput(
        "Constants/Intake/MetersPerMotorRotation_m", Mechanical.METERS_PER_MOTOR_ROTATION);

    // Simulation mechanical constants
    Logger.recordOutput("Constants/Intake/CarriageMass_kg", Mechanical.CARRIAGE_MASS_KG);
    Logger.recordOutput("Constants/Intake/DrumRadius_m", Mechanical.DRUM_RADIUS_METERS);
    Logger.recordOutput("Constants/Intake/MaxTravel_m", Mechanical.MAX_TRAVEL_METERS);

    // Deadband
    Logger.recordOutput("Constants/Intake/PositionDeadband_m", Software.POSITION_DEADBAND);

    // Motion profile constraints (logged at startup with their default values)
    Logger.recordOutput(
        "Constants/Intake/PID/Real/MaxVelocity_mps", PID.Real.MAX_VELOCITY_MPS.get());
    Logger.recordOutput(
        "Constants/Intake/PID/Real/MaxAcceleration_mpss", PID.Real.MAX_ACCELERATION_MPSS.get());
    Logger.recordOutput("Constants/Intake/PID/Sim/MaxVelocity_mps", PID.Sim.MAX_VELOCITY_MPS.get());
    Logger.recordOutput(
        "Constants/Intake/PID/Sim/MaxAcceleration_mpss", PID.Sim.MAX_ACCELERATION_MPSS.get());

    // Current limits
    Logger.recordOutput("Constants/Intake/CurrentLimit/LinearSmart_A", CurrentLimits.LINEAR_SMART);
    Logger.recordOutput(
        "Constants/Intake/CurrentLimit/LinearSecondary_A", CurrentLimits.LINEAR_SECONDARY);
    Logger.recordOutput("Constants/Intake/CurrentLimit/RollerSmart_A", CurrentLimits.ROLLER_SMART);
    Logger.recordOutput(
        "Constants/Intake/CurrentLimit/RollerSecondary_A", CurrentLimits.ROLLER_SECONDARY);

    // FuelSim constants
    Logger.recordOutput("Constants/Intake/FuelSim/BoundingBoxWidth_m", FuelSim.WIDTH.in(Meters));
    Logger.recordOutput("Constants/Intake/FuelSim/BoundingBoxLength_m", FuelSim.LENGTH.in(Meters));

    // Visualization constants
    Logger.recordOutput(
        "Constants/Intake/Visualization/SlideAngle_deg", Mechanical.SLIDE_ANGLE_DEGREES);
    Logger.recordOutput("Constants/Intake/Visualization/MaxTravel_m", Mechanical.MAX_TRAVEL_METERS);
  }
}
