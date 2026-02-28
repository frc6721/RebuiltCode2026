package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating feeder-related commands.
 *
 * <p>Provides commands for:
 *
 * <ul>
 *   <li><b>Open-loop:</b> Duty cycle control for simple testing
 *   <li><b>Closed-loop:</b> PID velocity control with feedforward for precise feeding
 *   <li><b>Characterization:</b> Automatic kS/kV feedforward constant discovery
 * </ul>
 *
 * <p>Used in shooting sequences: shooter spins up -> wait for speed -> feed -> stop.
 */
public class FeederCommands {
  /** Deadband threshold for joystick input - values below this are treated as zero. */
  private static final double DEADBAND = 0.1;

  // Feedforward characterization tuning constants
  private static final double FF_START_DELAY = 2.0; // seconds to wait for idle before ramp
  private static final double FF_RAMP_RATE = 0.1; // Volts per second

  // ==================== OPEN-LOOP (DUTY CYCLE) COMMANDS ====================



  // ==================== CLOSED-LOOP (VELOCITY) COMMANDS ====================

  /**
   * Creates a command to set the feeder to a specific target velocity using PID + feedforward. Sets
   * speed once and finishes immediately - the motor controller's PID + Motion Magic maintains the
   * speed afterward.
   *
   * <p>Use {@link #waitForFeederToReachSpeed(Feeder)} before proceeding if you need the feeder at
   * speed.
   *
   * @param feeder The feeder subsystem
   * @param speed The target feeder speed (use RPM.of(), RadiansPerSecond.of(), etc.)
   * @return A command that sets the feeder target speed once and finishes immediately
   */
  public static Command setFeederTargetSpeed(Feeder feeder, AngularVelocity speed) {
    return Commands.runOnce(
        () -> {
          feeder.setFeederVelocity(speed);
        },
        feeder);
  }

  /**
   * Creates a command to continuously run the feeder at a target velocity. Unlike {@link
   * #setFeederTargetSpeed}, this command runs every 20ms and does NOT finish on its own - use with
   * {@code whileTrue()} or cancel it manually.
   *
   * @param feeder The feeder subsystem
   * @param speed The target feeder speed
   * @return A command that continuously maintains the feeder at the target velocity
   */
  public static Command runFeederAtVelocity(Feeder feeder, AngularVelocity speed) {
    return Commands.run(
            () -> {
              feeder.setFeederVelocity(speed);
            },
            feeder)
        .withName("RunFeederAtVelocity");
  }

  /**
   * Creates a command that waits until the feeder reaches its target speed (within tolerance).
   *
   * <p>The tolerance is configured in {@link
   * frc.robot.subsystems.feeder.FeederConstants.Software#PID_TOLERANCE}. Consider adding a timeout:
   * {@code .withTimeout(2.0)}.
   *
   * @param feeder The feeder subsystem
   * @return A command that finishes when the feeder reaches target speed
   */
  public static Command waitForFeederToReachSpeed(Feeder feeder) {
    return Commands.waitUntil(() -> feeder.areFeederWheelsAtTargetSpeed());
  }

  // ==================== STOP COMMAND ====================

  /**
   * Creates a command to stop the feeder motor and reset target speed. Uses runOnce() - executes
   * once and finishes immediately.
   *
   * @param feeder The feeder subsystem
   * @return A command that stops the feeder once and finishes
   */
  public static Command stopFeeder(Feeder feeder) {
    return Commands.runOnce(() -> feeder.stop(), feeder);
  }

  // ==================== CHARACTERIZATION COMMANDS ====================

  /**
   * Simple feedforward characterization for the feeder motors.
   *
   * <p>Ramps voltage up linearly while sampling the resulting angular velocity. When the command is
   * canceled (release the button), performs a least-squares linear regression to compute kS and kV
   * and prints the results to the console.
   *
   * <p><b>How to use:</b>
   *
   * <ol>
   *   <li>Bind this command to a button with {@code whileTrue()}
   *   <li>Hold the button - the feeder will slowly ramp up
   *   <li>Release the button when the feeder is running fast (but before it hits mechanical limits)
   *   <li>Check the console output for kS and kV values
   *   <li>Enter those values into {@link
   *       frc.robot.subsystems.feeder.FeederConstants.Feedforward.Real}
   * </ol>
   *
   * <p><b>Units:</b> kS is in Volts, kV is in Volts per RPM
   *
   * @param feeder The feeder subsystem
   * @return A characterization command that ramps voltage and computes kS/kV on cancel
   */
  public static Command feedforwardCharacterization(Feeder feeder) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data from any previous run
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Ensure idle for a moment so things settle (no residual motion)
        Commands.run(() -> feeder.runCharacterization(Volts.of(0.0)), feeder)
            .withTimeout(FF_START_DELAY),

        // Start the ramp timer
        Commands.runOnce(timer::restart),

        // Ramp voltage and sample velocity every 20ms
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  feeder.runCharacterization(Volts.of(voltage));

                  // Sample velocity in rad/s, convert to RPM for the regression
                  double velocityRadPerSec = feeder.getFFCharacterizationVelocity();
                  double velocityRPM = velocityRadPerSec * 60.0 / (2.0 * Math.PI);
                  velocitySamples.add(velocityRPM);
                  voltageSamples.add(voltage);
                },
                feeder)

            // When cancelled (button released), calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i); // x = velocity (RPM)
                    sumY += voltageSamples.get(i); // y = voltage (V)
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  // Least-squares linear fit: voltage = kS + kV * velocity
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Feeder FF Characterization Results **********");
                  System.out.println("\tkS (Volts): " + formatter.format(kS));
                  System.out.println("\tkV (Volts per RPM): " + formatter.format(kV));
                  System.out.println("Enter these values into FeederConstants.Feedforward.Real");
                }));
  }

public static Command runFeederAtVoltage(Feeder feeder, double voltage) {
    return Commands.runOnce(() -> feeder.runFeederAtVoltage(Volts.of(voltage)), feeder);
  }
}
