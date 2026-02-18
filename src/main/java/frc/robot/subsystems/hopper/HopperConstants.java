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
  }
}
