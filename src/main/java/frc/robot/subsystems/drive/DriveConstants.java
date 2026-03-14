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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // 15.1 ft/s converted to meters per second for a mk4i module with L2 gearing
  // TODO: Update with your max speed from your swerve modules documentation
  public static final double maxSpeedMetersPerSec = 4;
  public static final double odometryFrequency = 100.0; // Hz

  // TODO: update based on your robots properties
  public static final double trackWidth = Units.inchesToMeters(20.75);
  public static final double wheelBase = Units.inchesToMeters(26);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  // TODO: Update these values by following the instructions
  //   public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0); // module 0
  //   public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0); // module 1
  //   public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0); // module 2
  //   public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0); // module 3

  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-2.726169); // module 0
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-2.759878); // module 1
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-1.902592); // module 2
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-1.020192); // module 3

  // Device CAN IDs
  // TODO: Update CAN IDS
  public static final int pigeonCanId = 37;

  public static final int kFrontLeftDrivingCanId = 8;
  public static final int kFrontLeftTurningCanId = 7;

  public static final int kFrontRightDrivingCanId = 2;
  public static final int kFrontRightTurningCanId = 1;

  public static final int kRearLeftDrivingCanId = 5;
  public static final int kRearLeftTurningCanId = 6;

  public static final int kRearRightDrivingCanId = 4;
  public static final int kRearRightTurningCanId = 3;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 40;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  // TODO: Update based on your swerve module's documentation
  public static final double driveMotorReduction = 6.75;
  // TODO: Change the motor type if you are using a different motor for your drive motor
  public static final DCMotor driveGearbox = DCMotor.getNEO(1); // changed from Vortex

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      driveEncoderPositionFactor / 60.0; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  // TODO: Tune these after initial setup by following the instructions
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.1631;
  public static final double driveKv = 0.1358;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  // TODO: update based on your swerve modules's documenation
  public static final double turnMotorReduction = 155 / 7.0; // mk4i swerve module
  // TODO: Change to your motor type if you are not using a NEO motor for turning
  public static final DCMotor turnGearbox = DCMotor.getNEO(1); // change dfrom NEO550

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      turnEncoderPositionFactor / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  // TODO: Tune these after initial setup by following the instructions
  public static final double turnKp = 3.5;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // ── Turn-to-Angle (Heading Lock) PID constants ───────────────────────────
  // These are used by DriveCommands.joystickDriveAtAngle() to rotate the robot
  // to face a specific field-relative heading while the driver controls translation.
  //
  // kP: How aggressively to correct heading error (higher = snappier but may oscillate)
  // kD: Dampens oscillation (start at 0, increase if robot overshoots)
  // Max velocity: Maximum rotation speed the PID can command (rad/s)
  // Max acceleration: How quickly the PID can ramp rotation speed (rad/s²)
  //
  // TODO: Tune these on the real robot — start with kP and adjust if the robot
  //       overshoots (increase kD) or is too slow to snap to heading (increase kP).
  public static final double turnToAngleKP = 5.0;
  public static final double turnToAngleKD = 0.5;
  public static final double turnToAngleMaxVelocity = 8.0; // rad/s
  public static final double turnToAngleMaxAcceleration = 20.0; // rad/s²

  // PathPlanner configuration
  // TODO: Update these with your robots physical properties
  public static final double robotMassKg = 37.0; // ~80lbs
  public static final double robotMOI = 5.835;
  public static final double wheelCOF = 1.0;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
