package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FeederConstants {

  /** Mechanical properties of the feeder mechanism. */
  public static class Mechanical {
    /** Gear ratio from motor to feeder mechanism (motor rotations per mechanism rotation) */
    public static final double GEAR_RATIO = 4.0;

    /** Motor type for the feeder (2x NEO per side) */
    public static final DCMotor MOTOR = DCMotor.getNEO(2);

    /**
     * Moment of inertia for the feeder wheels and belt system. Estimated for 4" Thrifty Squish
     * Wheels + belt + pulleys. Adjust after characterization if sim doesn't match real behavior.
     */
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0025);
  }

  /** Motor configuration for the feeder. */
  public static class Motor {
    /**
     * Base motor inversion flag. The left motor uses this value, the right motor is always inverted
     * relative to the left so that both feed wheels physically spin the same direction.
     */
    public static final boolean INVERTED = false;
  }

  /**
   * Velocity and acceleration limits for the feeder wheels. These are used by the Motion Magic
   * controller on the SparkMax to smoothly accelerate/decelerate.
   */
  public static class Limits {
    /** Minimum feeder speed - below this the motor just stalls */
    public static final AngularVelocity MIN_SPEED = RPM.of(50);

    /** Maximum feeder speed - limited by NEO through 4:1 gearbox (~1400 RPM output) */
    public static final AngularVelocity MAX_SPEED = RPM.of(5000);

    /**
     * Maximum acceleration for Motion Magic profiling. Controls how quickly the feeder ramps up to
     * target speed. Higher = faster response but more current draw.
     */
    public static final AngularAcceleration MAX_ACCEL = RPM.per(Second).of(8000); // 2000 RPM/s
  }

  /**
   * PID constants for closed-loop velocity control. Tuned separately for real hardware and
   * simulation. Uses LoggedNetworkNumber so values can be adjusted live over NetworkTables.
   */
  public static class PID {
    /** Real robot PID values - tune these on the actual robot */
    public static class Real {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Feeder/FEEDER_PID/Real/kP", 0.000);
      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Feeder/FEEDER_PID/Real/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Feeder/FEEDER_PID/Real/kD", 0.000);
    }

    /** Simulation PID values */
    public static class Sim {
      public static final LoggedNetworkNumber KP =
          new LoggedNetworkNumber("Feeder/FEEDER_PID/Sim/kP", 0.0001);
      public static final LoggedNetworkNumber KI =
          new LoggedNetworkNumber("Feeder/FEEDER_PID/Sim/kI", 0.0);
      public static final LoggedNetworkNumber KD =
          new LoggedNetworkNumber("Feeder/FEEDER_PID/Sim/kD", 0.0);
    }
  }

  /**
   * Feedforward constants for velocity control. kS overcomes static friction, kV maps velocity to
   * voltage. Run the feedforward characterization command to find these values for your robot.
   */
  public static class Feedforward {
    /** Real robot feedforward values - run characterization to find these */
    public static class Real {
      /** Static friction voltage (voltage needed to overcome friction and start moving) */
      public static final LoggedNetworkNumber KS =
          new LoggedNetworkNumber("Feeder/FEEDER_FF/Real/kS", 0.19124); // was .25

      /** Velocity feedforward constant (Volts per RPM) */
      public static final LoggedNetworkNumber KV =
          new LoggedNetworkNumber("Feeder/FEEDER_FF/Real/kV", 0.00204); // was .002
    }

    /** Simulation feedforward values */
    public static class Sim {
      public static final LoggedNetworkNumber KS =
          new LoggedNetworkNumber("Feeder/FEEDER_FF/Sim/kS", 0.0);
      public static final LoggedNetworkNumber KV =
          new LoggedNetworkNumber("Feeder/FEEDER_FF/Sim/kV", 0.002);
    }
  }

  public static final Voltage DEFAULT_FEED_VOLTAGE = Volts.of(12);

  /**
   * Target velocities for feeder operations using closed-loop PID + feedforward control.
   *
   * <p>These replace the open-loop voltage presets in {@link Voltages}. Closed-loop velocity
   * control provides more consistent feeding regardless of battery voltage.
   */
  public static class Speeds {
    /**
     * Target feeder RPM during a shooting sequence. This is the closed-loop equivalent of {@link
     * Voltages#SHOOT_FEED_VOLTAGE}. Tune by running the feeder at different speeds and finding the
     * RPM that reliably pushes game pieces into the flywheel.
     */
    public static final AngularVelocity SHOOT_FEED_RPM = RPM.of(4000);
  }

  /**
   * Common voltage presets for feeder operations.
   *
   * <p>Positive voltage feeds game pieces toward the shooter. Negative voltage pushes them back
   * toward the intake (spit).
   */
  public static class Voltages {
    /** Voltage applied to feeder during a shooting sequence to push fuel into the flywheel. */
    public static final Voltage SHOOT_FEED_VOLTAGE = Volts.of(9);

    /** Voltage applied to feeder when spitting game pieces back out through the intake. */
    public static final Voltage SPIT_VOLTAGE = Volts.of(-6);
  }

  // ── Mode-selected getter methods ──
  // These return the appropriate constant based on whether we're running on real hardware or in
  // sim.

  /** Returns the appropriate PID kP based on current mode (real vs sim) */
  public static double getFeederKP() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KP.get() : PID.Real.KP.get();
  }

  /** Returns the appropriate PID kI based on current mode */
  public static double getFeederKI() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KI.get() : PID.Real.KI.get();
  }

  /** Returns the appropriate PID kD based on current mode */
  public static double getFeederKD() {
    return Constants.currentMode == Constants.Mode.SIM ? PID.Sim.KD.get() : PID.Real.KD.get();
  }
  /** Returns the appropriate feedforward kS based on current mode */
  public static double getFeederKS() {
    return Constants.currentMode == Constants.Mode.SIM
        ? Feedforward.Sim.KS.get()
        : Feedforward.Real.KS.get();
  }

  /** Returns the appropriate feedforward kV based on current mode */
  public static double getFeederKV() {
    return Constants.currentMode == Constants.Mode.SIM
        ? Feedforward.Sim.KV.get()
        : Feedforward.Real.KV.get();
  }

  /** Current limits for motor protection. */
  public static class CurrentLimits {
    /**
     * Set to {@code true} to enable current limiting on the feeder motors, or {@code false} to
     * disable it (useful during testing/characterization).
     */
    public static final boolean ENABLE_CURRENT_LIMITS = true;

    public static final int SMART_CURRENT_LIMIT = 80;
    public static final double SECONDARY_CURRENT_LIMIT = 100;
  }

  /** Software tuning settings for the feeder. */
  public static class Software {
    /**
     * Tolerance as a percentage (0.05 = 5%) for determining if feeder is at target speed. Used by
     * Feeder.areFeederWheelsAtTargetSpeed() to decide when it's safe to proceed.
     */
    public static final double PID_TOLERANCE = 0.05;
  }

  // Logging
  static {
    Logger.recordOutput("Constants/Feeder/GearRatio", Mechanical.GEAR_RATIO);
    Logger.recordOutput("Constants/Feeder/MOI_kgm2", Mechanical.MOI.in(KilogramSquareMeters));
    Logger.recordOutput("Constants/Feeder/MotorInverted", Motor.INVERTED);

    Logger.recordOutput("Constants/Feeder/CurrentLimit/Smart_A", CurrentLimits.SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Feeder/CurrentLimit/Secondary_A", CurrentLimits.SECONDARY_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Feeder/CurrentLimit/Enabled", CurrentLimits.ENABLE_CURRENT_LIMITS);

    Logger.recordOutput("Constants/Feeder/SpeedTolerance_pct", Software.PID_TOLERANCE);
  }
}
