package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeConstants {

  /** Hardware configuration for encoders and motor inversions. */
  public static class Hardware {
    /** Absolute encoder zero position (intake fully stowed position) */
    public static final Rotation2d PIVOT_ZERO_ROTATION = new Rotation2d(Degrees.of(-140));

    /** Absolute encoder configuration */
    public static final boolean PIVOT_ENCODER_INVERTED = true;

    public static final double PIVOT_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
    public static final double PIVOT_ENCODER_VELOCITY_FACTOR =
        (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    /** Motor inversions */
    public static final boolean RIGHT_PIVOT_INVERTED = true;

    public static final boolean ROLLER_INVERTED = false;
  }

  /** Physical/mechanical properties of the intake. */
  public static class Mechanical {
    /** Gear ratios - motor rotations per mechanism rotation */
    public static final double PIVOT_GEAR_RATIO = 16.0; // 16:1 reduction

    public static final double ROLLER_GEAR_RATIO = 1.0; // Direct drive

    /** Arm length from pivot point to center of mass (13 inches) */
    public static final Distance ARM_LENGTH = Inches.of(13.0);

    /**
     * Moment of inertia for the intake arm. Calculated for ~15 lbs (6.8 kg) concentrated at end of
     * 13" (0.33m) arm. MOI = m * r^2 = 6.8 * 0.33^2 ≈ 0.74 kg⋅m²
     */
    public static final MomentOfInertia PIVOT_MOI = KilogramSquareMeters.of(0.74);

    /** Angle limits - Zero degrees is stowed (up), positive is towards ground */
    public static final double MIN_ANGLE_DEGREES = -1.0;

    public static final double MAX_ANGLE_DEGREES = 91.0;
    public static final double STARTING_ANGLE_DEGREES = 0.0; // Starts stowed

    /** Motor type for the pivot (2x NEO in leader-follower) */
    public static final DCMotor PIVOT_MOTOR = DCMotor.getNEO(2);

    /** Motor type for the roller (1x NEO) */
    public static final DCMotor ROLLER_MOTOR = DCMotor.getNEO(1);
  }

  /** Position setpoints for the intake pivot. */
  public static class Positions {
    /**
     * Position setpoints in degrees. Larger angle is towards the ground, smaller angle is towards
     * the robot.
     */
    public static final LoggedNetworkNumber PICKUP =
        new LoggedNetworkNumber("Intake/Position/Pickup", 88);

    public static final LoggedNetworkNumber STOW =
        new LoggedNetworkNumber("Intake/Position/Stow", 0);
  }

  /** PID and feedforward tuning constants. */
  public static class PID {
    /** Real robot PID values - tuned for actual hardware */
    public static class Real {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Intake/Pivot/PID/Real/kP", 0.03);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Intake/Pivot/PID/Real/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Intake/Pivot/PID/Real/kD", 0.0);

      /**
       * Feedforward voltage multiplied by cos(angle) to compensate for gravity. When horizontal
       * (0°), full feedforward applies. When vertical (90°), no feedforward needed.
       */
      public static final double FEEDFORWARD = 0.0;
    }

    /** Simulation PID values - tuned for physics simulation */
    public static class Sim {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Intake/Pivot/PID/Sim/kP", 0.2);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Intake/Pivot/PID/Sim/kI", 0.0002);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Intake/Pivot/PID/Sim/kD", 0.0015);

      /** Simulation feedforward for gravity compensation */
      public static final double FEEDFORWARD = 2;
    }
  }

  /** Returns the appropriate PID kP based on current mode */
  public static double getPivotKP() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KP.get() : PID.Real.KP.get();
  }

  /** Returns the appropriate PID kI based on current mode */
  public static double getPivotKI() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KI.get() : PID.Real.KI.get();
  }

  /** Returns the appropriate PID kD based on current mode */
  public static double getPivotKD() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KD.get() : PID.Real.KD.get();
  }

  /** Returns the appropriate feedforward based on current mode */
  public static double getPivotFeedforward() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.FEEDFORWARD : PID.Real.FEEDFORWARD;
  }

  /** Current limits for motor protection. */
  public static class CurrentLimits {
    public static final int PIVOT_SMART = 40;
    public static final double PIVOT_SECONDARY = 55;
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
    /** Deadband for considering pivot "at position" in degrees */
    public static final double PIVOT_DEADBAND = 4.0;
  }

  /** 3D visualization constants for AdvantageScope. */
  public static class Visualization {
    /**
     * 3D offset from robot origin to intake pivot point. TODO: Update these values based on CAD or
     * measurements. X = forward/back, Y = up/down, Z = left/right (meters)
     */
    public static final Translation3d OFFSET =
        new Translation3d(
            Inches.of(-8.5).in(Meters), Inches.of(0).in(Meters), Inches.of(10).in(Meters));

    public static final Rotation3d ROTATION = new Rotation3d(0, -Math.PI / 2, Math.PI);

    /** Visualization arm length in meters for Mechanism2d display */
    public static final double ARM_LENGTH = Mechanical.ARM_LENGTH.in(Meters);
  }

  /** FuelSim bounding box constants for intake pickup simulation. */
  public static class FuelSim {
    /** Intake bounding box dimensions (10" deep, 20" wide). */
    public static final Distance WIDTH = Inches.of(10.0);

    public static final Distance LENGTH = Inches.of(20.0);
  }

  // Logging
  static {
    // Pivot angle limits
    Logger.recordOutput("Constants/Intake/PivotAngle/Min_deg", Mechanical.MIN_ANGLE_DEGREES);
    Logger.recordOutput("Constants/Intake/PivotAngle/Max_deg", Mechanical.MAX_ANGLE_DEGREES);

    // Gear ratios
    Logger.recordOutput("Constants/Intake/GearRatio/Pivot", Mechanical.PIVOT_GEAR_RATIO);
    Logger.recordOutput("Constants/Intake/GearRatio/Roller", Mechanical.ROLLER_GEAR_RATIO);

    // Mechanical properties
    Logger.recordOutput("Constants/Intake/ArmLength_m", Mechanical.ARM_LENGTH.in(Meters));
    Logger.recordOutput(
        "Constants/Intake/PivotMOI_kgm2", Mechanical.PIVOT_MOI.in(KilogramSquareMeters));
    Logger.recordOutput("Constants/Intake/Deadband_deg", Software.PIVOT_DEADBAND);

    // Current limits
    Logger.recordOutput("Constants/Intake/CurrentLimit/PivotSmart_A", CurrentLimits.PIVOT_SMART);
    Logger.recordOutput(
        "Constants/Intake/CurrentLimit/PivotSecondary_A", CurrentLimits.PIVOT_SECONDARY);
    Logger.recordOutput("Constants/Intake/CurrentLimit/RollerSmart_A", CurrentLimits.ROLLER_SMART);
    Logger.recordOutput(
        "Constants/Intake/CurrentLimit/RollerSecondary_A", CurrentLimits.ROLLER_SECONDARY);

    // FuelSim constants
    Logger.recordOutput("Constants/Intake/FuelSim/BoundingBoxWidth_m", FuelSim.WIDTH.in(Meters));
    Logger.recordOutput("Constants/Intake/FuelSim/BoundingBoxLength_m", FuelSim.LENGTH.in(Meters));
  }
}
