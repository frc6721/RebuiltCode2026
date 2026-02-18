package frc.robot.subsystems.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RealShooterIO implements ShooterIO {
  private SparkMax _flywheelMotor;

  public RealShooterIO() {
    configFlywheelMotor();
  }

  public void configFlywheelMotor() {
    _flywheelMotor = new SparkMax(Constants.CanIds.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ShooterConstants.Mechanical.INVERTED)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CurrentLimits.SMART)
        .secondaryCurrentLimit(ShooterConstants.CurrentLimits.SECONDARY)
        .voltageCompensation(12.0);
    config.closedLoop.pid(
        ShooterConstants.getFlywheelKP(),
        ShooterConstants.getFlywheelKI(),
        ShooterConstants.getFlywheelKD());

    config.closedLoop.maxMotion.maxAcceleration(
        ShooterConstants.Limits.MAX_ACCEL.in(RotationsPerSecond.per(Second)) * 60.0);

    tryUntilOk(
        _flywheelMotor,
        5,
        () ->
            _flywheelMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(ShooterIOInputs inputs) {
    // Flywheel motor
    inputs._flywheelMotorTemperature = Celsius.of(_flywheelMotor.getMotorTemperature());
    inputs._flywheelMotorVelocity =
        RotationsPerSecond.of(_flywheelMotor.getEncoder().getVelocity() / 60.0);
    inputs._flywheelMotorVoltage =
        Volts.of(_flywheelMotor.getAppliedOutput() * _flywheelMotor.getBusVoltage());
    inputs._flywheelMotorCurrent = Amps.of(_flywheelMotor.getOutputCurrent());
  }

  // Flywheel motor methods
  public void setFlywheelSpeed(AngularVelocity speed) {
    double targetRPM = speed.in(RotationsPerSecond) * 60.0;

    // Manually calculate feedforward voltage: V = kS * sign(velocity) + kV * velocity
    double kS = ShooterConstants.getFlywheelKS();
    double kV = ShooterConstants.getFlywheelKV();
    double ffVolts = kS * Math.signum(targetRPM) + kV * targetRPM;

    _flywheelMotor
        .getClosedLoopController()
        .setReference(
            targetRPM,
            ControlType.kMAXMotionVelocityControl,
            com.revrobotics.spark.ClosedLoopSlot.kSlot0,
            ffVolts,
            com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  public void setFlyWheelDutyCycle(double output) {
    this._flywheelMotor.set(output);
  }

  public void stopFlywheel() {
    // reset the integral accumulator to prevent integral windup
    _flywheelMotor.getClosedLoopController().setIAccum(0);
    _flywheelMotor.stopMotor();
  }

  public void setFlywheelVoltage(Voltage volts) {
    _flywheelMotor.setVoltage(volts.magnitude());
  }
}
