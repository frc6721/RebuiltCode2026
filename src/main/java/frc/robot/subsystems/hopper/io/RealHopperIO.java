package frc.robot.subsystems.hopper.io;

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
import frc.robot.subsystems.hopper.HopperConstants;

/**
 * Real hardware implementation of HopperIO using a single SparkMax motor controller.
 *
 * <p>The hopper is a simple open-loop mechanism - it runs forward, backward, or stops. No PID or
 * feedback control is needed.
 */
public class RealHopperIO implements HopperIO {
  private final SparkMax _hopperMotor;

  public RealHopperIO() {
    _hopperMotor = new SparkMax(Constants.CanIds.HOPPER_MOTOR_ID, MotorType.kBrushless);
    configHopperMotor();
  }

  /** Configures the hopper motor with current limits and idle mode. */
  public void configHopperMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(HopperConstants.Motor.INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HopperConstants.CurrentLimits.SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(HopperConstants.CurrentLimits.SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    tryUntilOk(
        _hopperMotor,
        5,
        () ->
            _hopperMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs._hopperMotorTemperature = Celsius.of(_hopperMotor.getMotorTemperature());
    inputs._hopperMotorVoltage =
        Volts.of(_hopperMotor.getAppliedOutput() * _hopperMotor.getBusVoltage());
    inputs._hopperMotorCurrent = Amps.of(_hopperMotor.getOutputCurrent());
  }

  @Override
  public void setMotorSpeed(double speed) {
    _hopperMotor.set(speed);
  }
}
