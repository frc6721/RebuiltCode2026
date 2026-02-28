package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterConstants {

  /** Physical/mechanical properties of the flywheel. */
  public static class Mechanical {
    /** Gear ratio from motor to flywheel (motor rotations per flywheel rotation) */
    public static final double GEAR_RATIO = 1.0;

    /** Flywheel diameter (4" urethane wheel) */
    public static final Distance DIAMETER = Inches.of(4.0);

    /**
     * Moment of inertia for the flywheel. Estimated for 4" urethane wheel + custom aluminum
     * flywheel. A typical 4" wheel + aluminum disc is approximately 0.003-0.005 kg⋅m²
     */
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0004);

    /** Motor type for the flywheel (1x NEO Vortex per side, used with SparkFlex) */
    public static final DCMotor MOTOR = DCMotor.getNeoVortex(1);

    /**
     * Base motor inversion flag. The left motor uses this value, the right motor is always inverted
     * relative to the left so that both flywheels physically spin the same direction.
     */
    public static final boolean INVERTED = false;
  }

  /** Velocity and acceleration limits for the flywheel. */
  public static class Limits {
    public static final AngularVelocity MIN_SPEED = RPM.of(100);

    public static final AngularVelocity MAX_SPEED = RPM.of(5600);

    public static final AngularAcceleration MAX_ACCEL = RPM.per(Second).of(3000); // 3000 RPM/s
  }

  /** PID and feedforward tuning constants. */
  public static class PID {
    /** Real robot PID values - tuned for actual hardware (on motor controller) */
    public static class Real {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kP", 0.00025);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kD", 0.000175);
      public static final LoggedNetworkNumber FF =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kFF", 0.000000);
    }

    /** Simulation PID values */
    public static class Sim {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kP", 0.0001);

      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kD", 0.0);
      public static final LoggedNetworkNumber FF =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kV", 0.0);
    }
  }

  /** Feedforward constants for velocity control. */
  public static class Feedforward {
    /** Real robot feedforward values */
    public static class Real {
      /** Static friction voltage (voltage to overcome friction) */
      public static final LoggedNetworkNumber KS =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Real/kS", 0.26750);

      /** Velocity feedforward constant (Volts per RPM) */
      public static final LoggedNetworkNumber KV =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Real/kV", 0.00170);
    }

    /** Simulation feedforward values */
    public static class Sim {
      public static final LoggedNetworkNumber KS =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Sim/kS", 0.0);
      public static final LoggedNetworkNumber KV =
          new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Sim/kV", 0.0018);
    }
  }

  // Mode-selected getters

  /** Returns the appropriate PID kP based on current mode */
  public static double getFlywheelKP() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KP.get() : PID.Real.KP.get();
  }

  /** Returns the appropriate PID kI based on current mode */
  public static double getFlywheelKI() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KI.get() : PID.Real.KI.get();
  }

  /** Returns the appropriate PID kD based on current mode */
  public static double getFlywheelKD() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KD.get() : PID.Real.KD.get();
  }

  /** Returns the appropriate PID FF based on current mode */
  public static double getFlywheelFF() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.FF.get() : PID.Real.FF.get();
  }

  /** Returns the appropriate feedforward kS based on current mode */
  public static double getFlywheelKS() {
    return Constants.currentMode == Constants.Mode.SIM
        ? Feedforward.Sim.KS.get()
        : Feedforward.Real.KS.get();
  }

  /** Returns the appropriate feedforward kV based on current mode */
  public static double getFlywheelKV() {
    return Constants.currentMode == Constants.Mode.SIM
        ? Feedforward.Sim.KV.get()
        : Feedforward.Real.KV.get();
  }

  /** Current limits for motor protection. */
  public static class CurrentLimits {
    public static final int SMART = 100;
    public static final double SECONDARY = 100;
  }

  /** Software tuning settings. */
  public static class Software {
    /** Tolerance as a percentage (0.05 = 5%) for determining if flywheel is at target speed */
    public static final double PID_TOLERANCE = 0.05;

    /** Idle duty cycle when shooter is not actively shooting */
    public static final double IDLE_DUTY_CYCLE = 0.0;
  }

  /** 3D visualization constants for AdvantageScope. */
  public static class Visualization {
    /**
     * 3D offset from robot origin to flywheel center. TODO: Update based on CAD or measurements.
     */
    public static final Translation3d OFFSET =
        new Translation3d(
            Inches.of(26 / 2).in(Meters), Inches.of(20).in(Meters), Inches.of(6).in(Meters));

    /** 3D rotation offset for visualization orientation */
    public static final Rotation3d ROTATION = new Rotation3d(0.0, 0.0, 0.0);

    /** Visualization flywheel radius in meters for Mechanism2d display */
    public static final double FLYWHEEL_RADIUS = Mechanical.DIAMETER.in(Meters) / 2.0;
  }

  /** Distance-to-speed lookup table for automatic shooting. */
  public static class DistanceMap {
    /**
     * Maps distance to target (meters) → required shooter speed (RPM). Characterize by shooting
     * from various distances and recording the RPM needed.
     */
    public static final InterpolatingDoubleTreeMap SPEED_MAP = new InterpolatingDoubleTreeMap();

    static {
      SPEED_MAP.put(Meters.of(1.0).in(Meters), RPM.of(1000.0).in(RPM));
      SPEED_MAP.put(Meters.of(3.0).in(Meters), RPM.of(1500.0).in(RPM));
    }
  }

  /** FuelSim constants for shooter trajectory visualization. */
  public static class FuelSim {
    /** Height of the shooter exit point above the ground. */
    public static final Distance HEIGHT_FROM_GROUND = Inches.of(18.0);

    /** Forward offset of shooter from robot center. */
    public static final Distance FORWARD_OFFSET = Inches.of(8.0);

    /** Side offset of shooter from robot center. */
    public static final Distance SIDE_OFFSET = Inches.of(0.0);

    /** Fixed hood angle from horizontal. */
    public static final Angle HOOD_ANGLE = Degrees.of(70.0);

    /** Minimum flywheel RPM threshold to trigger fuel launch visualization. */
    public static final AngularVelocity RPM_THRESHOLD_FOR_LAUNCH = RPM.of(200.0);

    /** Time between consecutive fuel launches. */
    public static final Time LAUNCH_INTERVAL = Seconds.of(0.5);

    /** Diameter of the shooter wheel for velocity conversion. */
    public static final Distance WHEEL_DIAMETER = Mechanical.DIAMETER;

    /** Maximum number of fuel pieces the robot can hold. */
    public static final int MAX_HOPPER_CAPACITY = 30;

    /** Number of fuel pieces robot starts with when enabled. */
    public static final int STARTING_FUEL_COUNT = 8;

    /** Number of points used to render the trajectory visualization. */
    public static final int TRAJECTORY_POINTS = 50;

    /**
     * Total time span (in seconds) for the trajectory visualization. Increase if trajectory ends
     * mid-air before hitting the ground.
     */
    public static final double TRAJECTORY_TIME_SPAN_SECONDS = 2.0;
  }

  // Logging
  static {
    Logger.recordOutput("Constants/Shooter/FlywheelGearRatio", Mechanical.GEAR_RATIO);
    Logger.recordOutput(
        "Constants/Shooter/FlywheelMOI_kgm2", Mechanical.MOI.in(KilogramSquareMeters));
    Logger.recordOutput("Constants/Shooter/SpeedTolerance_pct", Software.PID_TOLERANCE);
    Logger.recordOutput("Constants/Shooter/FlywheelInverted", Mechanical.INVERTED);

    Logger.recordOutput("Constants/Shooter/CurrentLimit/Smart_A", CurrentLimits.SMART);
    Logger.recordOutput("Constants/Shooter/CurrentLimit/Secondary_A", CurrentLimits.SECONDARY);

    Logger.recordOutput(
        "Constants/Shooter/FuelSim/Height_m", FuelSim.HEIGHT_FROM_GROUND.in(Meters));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/ForwardOffset_m", FuelSim.FORWARD_OFFSET.in(Meters));
    Logger.recordOutput("Constants/Shooter/FuelSim/SideOffset_m", FuelSim.SIDE_OFFSET.in(Meters));
    Logger.recordOutput("Constants/Shooter/FuelSim/HoodAngle_deg", FuelSim.HOOD_ANGLE.in(Degrees));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/RPMThreshold_RPM", FuelSim.RPM_THRESHOLD_FOR_LAUNCH.in(RPM));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/LaunchInterval_s", FuelSim.LAUNCH_INTERVAL.in(Seconds));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/WheelDiameter_m", FuelSim.WHEEL_DIAMETER.in(Meters));
    Logger.recordOutput("Constants/Shooter/FuelSim/MaxHopperCapacity", FuelSim.MAX_HOPPER_CAPACITY);
    Logger.recordOutput("Constants/Shooter/FuelSim/StartingFuelCount", FuelSim.STARTING_FUEL_COUNT);
    Logger.recordOutput("Constants/Shooter/FuelSim/TrajectoryPoints", FuelSim.TRAJECTORY_POINTS);
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/TrajectoryTimeSpan_s", FuelSim.TRAJECTORY_TIME_SPAN_SECONDS);
  }
}
