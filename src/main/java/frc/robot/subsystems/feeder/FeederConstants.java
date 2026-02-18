package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.Logger;

public class FeederConstants {

  /** Mechanical properties of the feeder mechanism. */
  public static class Mechanical {
    /** Gear ratio from motor to feeder mechanism (motor rotations per mechanism rotation) */
    public static final double GEAR_RATIO = 4.0; // 4:1 AM Sport Gearbox

    /** Motor type for the feeder (1x NEO) */
    public static final DCMotor MOTOR = DCMotor.getNEO(1);
  }

  /** Motor configuration for the feeder. */
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
    Logger.recordOutput("Constants/Feeder/GearRatio", Mechanical.GEAR_RATIO);
    Logger.recordOutput("Constants/Feeder/MotorInverted", Motor.INVERTED);

    Logger.recordOutput("Constants/Feeder/CurrentLimit/Smart_A", CurrentLimits.SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Feeder/CurrentLimit/Secondary_A", CurrentLimits.SECONDARY_CURRENT_LIMIT);
  }
}
