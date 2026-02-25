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
    public static final boolean LINEAR_MOTOR_INVERTED = false;

    /** Roller motor inversion */
    public static final boolean ROLLER_INVERTED = false;
  }

  /** Physical/mechanical properties of the intake. */
  public static class Mechanical {
    /** Gear ratio for the linear slide motor (motor rotations per unit of travel) */
    public static final double LINEAR_GEAR_RATIO = 16.0;

    /** Roller gear ratio */
    public static final double ROLLER_GEAR_RATIO = 1.0;

    /**
     * Position conversion factor for the internal encoder. Converts motor rotations to linear
     * position in rotations at the output. Adjust this based on your lead screw pitch or belt/gear
     * ratio so that the encoder position reads in meaningful units.
     */
    public static final double LINEAR_POSITION_CONVERSION_FACTOR = 1.0;

    /**
     * Velocity conversion factor for the internal encoder. Converts motor RPM to output velocity.
     */
    public static final double LINEAR_VELOCITY_CONVERSION_FACTOR = 1.0 / 60.0;

    /** Motor type for the linear slide (1x NEO) */
    public static final DCMotor LINEAR_MOTOR = DCMotor.getNEO(1);

    /** Motor type for the roller (1x NEO) */
    public static final DCMotor ROLLER_MOTOR = DCMotor.getNEO(1);

    /**
     * Mass of the intake carriage (roller assembly) in kilograms. Used by ElevatorSim in simulation
     * to model inertia.
     */
    public static final double CARRIAGE_MASS_KG = 5.0;

    /**
     * Effective drum radius in meters that converts between motor rotations and linear travel.
     *
     * <p>This value ties "output rotations" (what the encoder reads) to real linear distance via
     * {@code distance = rotations × 2π × drumRadius}. Derived from the max travel of the slide (11
     * in ≈ 0.2794 m) and the max output rotations (10.0):
     *
     * <pre>drumRadius = maxTravel / (maxRotations × 2π)</pre>
     */
    public static final double DRUM_RADIUS_METERS =
        Inches.of(11).in(Meters) / (10.0 * 2.0 * Math.PI);

    /**
     * Maximum linear travel of the intake slide in meters. Used as the upper bound for ElevatorSim
     * so the simulated position stays within the physical range.
     */
    public static final double MAX_TRAVEL_METERS = Inches.of(11).in(Meters);

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
   * Position setpoints for the linear slide.
   *
   * <p>Position 0 = fully retracted (in). Positive values = extending out. Units are in output
   * rotations (after gear ratio). Tune these to match your hardware.
   */
  public static class Positions {
    /** Fully retracted (stowed) position */
    public static final LoggedNetworkNumber RETRACTED =
        new LoggedNetworkNumber("Intake/Position/Retracted", 0.0);

    /** Fully extended (deployed) position */
    public static final LoggedNetworkNumber EXTENDED =
        new LoggedNetworkNumber("Intake/Position/Extended", 10.0);
  }

  /** PID tuning constants for the linear slide position control. */
  public static class PID {
    /** Real robot PID values - tuned for actual hardware */
    public static class Real {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/kP", 0.1);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Intake/Linear/PID/Real/kD", 0.0);
    }

    /** Simulation PID values - tuned for physics simulation */
    public static class Sim {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/kP", 10);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Intake/Linear/PID/Sim/kD", 0.01);
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

  /** Current limits for motor protection. */
  public static class CurrentLimits {
    public static final int LINEAR_SMART = 40;
    public static final double LINEAR_SECONDARY = 55;
    public static final int ROLLER_SMART = 50;
    public static final double ROLLER_SECONDARY = 60;
  }

  /** Roller motor settings. */
  public static class Roller {
    public static final LoggedNetworkNumber ACQUIRE_SPEED =
        new LoggedNetworkNumber("Intake/Roller/Acquire Speed", 1);

    public static final LoggedNetworkNumber CURRENT_CUTOFF =
        new LoggedNetworkNumber("Intake/Roller/Current Cutoff", 40);
  }

  /** Software tuning settings. */
  public static class Software {
    /** Deadband for considering linear slide "at position" (in output rotations) */
    public static final double POSITION_DEADBAND = 0.5;
  }

  /** FuelSim bounding box constants for intake pickup simulation. */
  public static class FuelSim {
    /** Intake bounding box dimensions */
    public static final Distance WIDTH = Inches.of(10.0);

    public static final Distance LENGTH = Inches.of(20.0);
  }

  // Logging
  static {
    // Gear ratios
    Logger.recordOutput("Constants/Intake/GearRatio/Linear", Mechanical.LINEAR_GEAR_RATIO);
    Logger.recordOutput("Constants/Intake/GearRatio/Roller", Mechanical.ROLLER_GEAR_RATIO);

    // Simulation mechanical constants
    Logger.recordOutput("Constants/Intake/Sim/CarriageMass_kg", Mechanical.CARRIAGE_MASS_KG);
    Logger.recordOutput("Constants/Intake/Sim/DrumRadius_m", Mechanical.DRUM_RADIUS_METERS);
    Logger.recordOutput("Constants/Intake/Sim/MaxTravel_m", Mechanical.MAX_TRAVEL_METERS);

    // Deadband
    Logger.recordOutput("Constants/Intake/PositionDeadband", Software.POSITION_DEADBAND);

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
