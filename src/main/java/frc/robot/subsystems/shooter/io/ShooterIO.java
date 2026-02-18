package frc.robot.subsystems.shooter.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    // Flywheel motor
    public Temperature _flywheelMotorTemperature;
    public AngularVelocity _flywheelMotorVelocity;
    public Voltage _flywheelMotorVoltage;
    public Current _flywheelMotorCurrent;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  // Flywheel motor methods
  public default void setFlywheelSpeed(AngularVelocity speed) {}

  public default void setFlyWheelDutyCycle(double output) {}

  public default void stopFlywheel() {}

  public default void setFlywheelVoltage(Voltage volts) {}
}
