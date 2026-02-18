// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

/**
 * This class defines the runtime mode used by AdvantageKit, hardware constants (CAN IDs, camera
 * counts), and robot physical dimensions.
 *
 * <p>The mode is always "real" when running on a roboRIO. Change the value of "simMode" to switch
 * between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Used in the hopper subsystem to determine whether to use an infinite hopper or not. This is
  // used for simulations. The virtual hopper is not used for the real robot.
  public static final boolean INFINITE_HOPPER = true;

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  // ── Hardware Constants ──────────────────────────────────────────────────────

  /** Number of vision cameras on the robot. */
  public static final int NUMBER_OF_CAMERAS = 2;

  /** CAN bus IDs for all motors and sensors on the robot. */
  public static class CanIds {
    // Intake
    public static int RIGHT_PIVOT_MOTOR_ID = 20;
    public static int LEFT_PIVOT_MOTOR_ID = 21;
    public static int ROLLER_MOTOR_ID = 22;

    // Shooter
    public static int FLYWHEEL_MOTOR_ID = 31;

    // Feeder
    public static int FEEDER_MOTOR_ID = 32;

    // Climber
    public static int CLIMBER_MOTOR_ID = 42;
  }

  // ── Robot Dimensions ────────────────────────────────────────────────────────

  /**
   * Robot physical dimensions for FuelSim and collision detection.
   *
   * <p>These constants define the overall size of the robot including bumpers. They are used by
   * FuelSim to detect collisions between the robot and game pieces.
   *
   * <p><b>Robot dimensions:</b>
   *
   * <ul>
   *   <li>Frame: 26" × 26" (square frame)
   *   <li>Bumpers: 4" thick on all sides
   *   <li>Total: 34" × 34" with bumpers (0.8636 m)
   *   <li>Bumper height: 6" off ground at top
   * </ul>
   */
  public static final class Dimensions {

    // Private constructor prevents instantiation (utility class)
    private Dimensions() {}

    /**
     * Full robot width including bumpers (left to right, along Y-axis).
     *
     * <p>Calculation:
     *
     * <ul>
     *   <li>Robot frame: 26"
     *   <li>Bumpers on each side: 4" × 2 = 8"
     *   <li>Total: 26" + 8" = 34" = 0.8636 m
     * </ul>
     */
    public static final Distance ROBOT_WIDTH = Inches.of(34.0);

    /** Full robot length including bumpers (front to back, along X-axis). */
    public static final Distance ROBOT_LENGTH = Inches.of(26.0 + 3.75);

    /**
     * Height from ground to top of bumpers.
     *
     * <p>Bumpers are mounted in the bumper zone with the top at approximately 6" off the ground.
     * This is used by FuelSim to determine if game pieces can be pushed by the robot (pieces above
     * bumper height won't collide with the robot body).
     */
    public static final Distance BUMPER_HEIGHT = Inches.of(4.0);

    // Log dimensions at class load
    static {
      Logger.recordOutput("Constants/Robot/Width_in", ROBOT_WIDTH.in(Inches));
      Logger.recordOutput("Constants/Robot/Length_in", ROBOT_LENGTH.in(Inches));
      Logger.recordOutput("Constants/Robot/BumperHeight_in", BUMPER_HEIGHT.in(Inches));
    }
  }
}
