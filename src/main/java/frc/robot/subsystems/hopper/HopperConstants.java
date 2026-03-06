package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.Logger;

/**
 * Constants for the Hopper subsystem.
 *
 * <p>The hopper is a single-motor mechanism that moves game pieces within the robot, similar to the
 * feeder but upstream in the game piece path.
 */
public class HopperConstants {

  /** Mechanical properties of the hopper mechanism. */
  public static class Mechanical {
    /** Gear ratio from motor to hopper mechanism (motor rotations per mechanism rotation) */
    public static final double GEAR_RATIO = 4.0;

    /** Motor type for the hopper (1x NEO) */
    public static final DCMotor MOTOR = DCMotor.getNEO(1);
  }

  /** Motor configuration for the hopper. */
  public static class Motor {
    /** Motor inversion */
    public static final boolean INVERTED = false;
  }

  /**
   * Duty-cycle speed presets for common hopper operations.
   *
   * <p>All values are duty cycle: -1.0 (full reverse) to +1.0 (full forward). Positive = feed game
   * pieces toward the shooter. Negative = push game pieces back toward the intake.
   */
  public static class Speeds {
    /** Speed for feeding game pieces toward the shooter during a shooting sequence. */
    public static final double FEED_SPEED = 0.3;

    /** Speed for acquiring game pieces (moving them forward in the hopper). */
    public static final double ACQUIRE_SPEED = 0.5;

    /** Slower acquire speed for gentle game piece handling. */
    public static final double SLOW_ACQUIRE_SPEED = 0.2;

    /** Speed for spitting game pieces back out through the intake. */
    public static final double SPIT_SPEED = -0.5;

    /** Slower spit speed for gentle ejection. */
    public static final double SLOW_SPIT_SPEED = -0.2;
  }

  /** Current limits for motor protection. */
  public static class CurrentLimits {
    public static final int SMART_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 55;
  }

  // Logging
  static {
    Logger.recordOutput("Constants/Hopper/GearRatio", Mechanical.GEAR_RATIO);
    Logger.recordOutput("Constants/Hopper/MotorInverted", Motor.INVERTED);

    Logger.recordOutput("Constants/Hopper/CurrentLimit/Smart_A", CurrentLimits.SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Hopper/CurrentLimit/Secondary_A", CurrentLimits.SECONDARY_CURRENT_LIMIT);

    Logger.recordOutput("Constants/Hopper/Speeds/FeedSpeed", Speeds.FEED_SPEED);
    Logger.recordOutput("Constants/Hopper/Speeds/AcquireSpeed", Speeds.ACQUIRE_SPEED);
    Logger.recordOutput("Constants/Hopper/Speeds/SlowAcquireSpeed", Speeds.SLOW_ACQUIRE_SPEED);
    Logger.recordOutput("Constants/Hopper/Speeds/SpitSpeed", Speeds.SPIT_SPEED);
    Logger.recordOutput("Constants/Hopper/Speeds/SlowSpitSpeed", Speeds.SLOW_SPIT_SPEED);
  }
}
