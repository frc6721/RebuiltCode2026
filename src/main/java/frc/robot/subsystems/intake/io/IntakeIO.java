package frc.robot.subsystems.intake.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // Right pivot motor
    public Temperature _intakeRightPivotMotorTemperature;
    public AngularVelocity _intakeRightPivotMotorVelocity;
    public Rotation2d _intakeRightPivotMotorPosition;
    public Voltage _intakeRightPivotMotorVoltage;
    public Current _intakeRightPivotMotorCurrent;

    // Left pivot motor
    public Temperature _intakeLeftPivotMotorTemperature;
    public AngularVelocity _intakeLeftPivotMotorVelocity;
    public Rotation2d _intakeLeftPivotMotorPosition;
    public Voltage _intakeLeftPivotMotorVoltage;
    public Current _intakeLeftPivotMotorCurrent;

    // Roller motor
    public Temperature _intakeRollerMotorTemperature;
    public AngularVelocity _intakeRollerMotorVelocity;
    public Voltage _intakeRollerMotorVoltage;
    public Current _intakeRollerMotorCurrent;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  // Pivot motor methods
  public default void setPivotMotorVoltage(double volts) {}

  public default void setIntakePivotDutyCycleOutput(double output) {}

  // Roller motor methods
  public default void setRollerMotorOutput(double output) {}
}
