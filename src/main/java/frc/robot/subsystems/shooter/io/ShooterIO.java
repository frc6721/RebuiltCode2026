package frc.robot.subsystems.shooter.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    // Left flywheel motor (leader)
    public Temperature _leftFlywheelMotorTemperature;
    public AngularVelocity _leftFlywheelMotorVelocity;
    public Voltage _leftFlywheelMotorVoltage;
    public Current _leftFlywheelMotorCurrent;

    // Right flywheel motor (follower)
    public Temperature _rightFlywheelMotorTemperature;
    public AngularVelocity _rightFlywheelMotorVelocity;
    public Voltage _rightFlywheelMotorVoltage;
    public Current _rightFlywheelMotorCurrent;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  // Flywheel motor methods
  public default void setFlywheelSpeed(AngularVelocity speed) {}

  public default void setFlyWheelDutyCycle(double output) {}

  public default void stopFlywheel() {}

  public default void setFlywheelVoltage(Voltage volts) {}
}
