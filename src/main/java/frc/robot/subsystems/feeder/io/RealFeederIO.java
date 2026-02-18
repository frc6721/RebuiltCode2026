package frc.robot.subsystems.feeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederConstants;

public class RealFeederIO implements FeederIO {
  private SparkMax _feederMotor;

  public RealFeederIO() {
    configFeederMotor();
  }

  public void configFeederMotor() {
    _feederMotor = new SparkMax(Constants.CanIds.FEEDER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(FeederConstants.Motor.INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.CurrentLimits.SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(FeederConstants.CurrentLimits.SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    tryUntilOk(
        _feederMotor,
        5,
        () ->
            _feederMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Feeder motor
    inputs._feederMotorTemperature = Celsius.of(_feederMotor.getMotorTemperature());
    inputs._feederMotorVoltage =
        Volts.of(_feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage());
    inputs._feederMotorCurrent = Amps.of(_feederMotor.getOutputCurrent());
  }

  @Override
  public void setMotorSpeed(double speed) {
    _feederMotor.set(speed);
  }
}
