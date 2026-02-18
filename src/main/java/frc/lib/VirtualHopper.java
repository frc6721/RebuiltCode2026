package frc.lib;

import org.littletonrobotics.junction.Logger;

/**
 * A singleton class that manages virtual fuel (game piece) storage for simulation.
 *
 * <p>The VirtualHopper tracks how many fuel pieces the robot is "carrying" during simulation. This
 * allows the FuelSim system to visualize when the robot picks up and shoots game pieces.
 *
 * <p><b>Singleton Pattern:</b> This class uses the singleton pattern to ensure there's only one
 * hopper tracking fuel count across the entire robot code. Access it using {@link #getInstance()}.
 *
 * <p><b>Usage Example:</b>
 *
 * <pre>
 * // Get the singleton instance
 * VirtualHopper hopper = VirtualHopper.getInstance();
 *
 * // Check if we have fuel to shoot
 * if (hopper.hasFuel()) {
 *     hopper.removeFuel();
 *     // Launch the fuel...
 * }
 *
 * // Add fuel when intake collects a game piece
 * hopper.addFuel();
 * </pre>
 *
 * <p><b>Integration Points:</b>
 *
 * <ul>
 *   <li><b>Intake:</b> Calls {@link #addFuel()} when a game piece is collected
 *   <li><b>Shooter:</b> Calls {@link #removeFuel()} when a game piece is launched
 *   <li><b>Robot.java:</b> Calls {@link #setFuelCount(int)} at match start to initialize
 * </ul>
 *
 * <p>All changes to fuel count are logged to AdvantageKit for visualization.
 */
public class VirtualHopper {

  // Singleton instance - there's only ever one hopper
  private static VirtualHopper instance = null;

  // Current number of fuel pieces in the hopper
  private int fuelCount = 0;

  /**
   * Private constructor enforces singleton pattern.
   *
   * <p>By making the constructor private, we prevent other code from creating new VirtualHopper
   * instances. The only way to get a VirtualHopper is through {@link #getInstance()}.
   */
  private VirtualHopper() {
    // Log initial state
    logFuelCount();
  }

  /**
   * Returns the singleton instance of VirtualHopper.
   *
   * <p>This method uses lazy initialization - the instance is only created the first time this
   * method is called. All subsequent calls return the same instance.
   *
   * @return The single VirtualHopper instance
   */
  public static VirtualHopper getInstance() {
    if (instance == null) {
      instance = new VirtualHopper();
    }
    return instance;
  }

  /**
   * Returns the current number of fuel pieces in the hopper.
   *
   * @return The current fuel count (0 or greater)
   */
  public int getFuelCount() {
    return fuelCount;
  }

  /**
   * Directly sets the fuel count to a specific value.
   *
   * <p>This is typically used at the start of a match to initialize the robot with starting fuel,
   * or to reset the hopper during testing.
   *
   * @param count The new fuel count (should be >= 0)
   */
  public void setFuelCount(int count) {
    fuelCount = Math.max(0, count);
    logFuelCount();
  }

  /**
   * Adds one fuel piece to the hopper.
   *
   * <p>Called by the intake when a game piece is successfully collected from the field. The fuel
   * count is incremented by 1 and logged for visualization.
   *
   * <p><b>Note:</b> This method does NOT check capacity limits. The caller (intake subsystem)
   * should verify there's room before calling this method.
   */
  public void addFuel() {
    fuelCount++;
    logFuelCount();
  }

  /**
   * Removes one fuel piece from the hopper (if available).
   *
   * <p>Called by the shooter when launching a game piece. The fuel count is decremented by 1 only
   * if there's at least one fuel in the hopper.
   *
   * <p><b>Safety Check:</b> This method will not decrement below zero. If the hopper is already
   * empty, calling this method has no effect.
   *
   * <p><b>Infinite Hopper Mode:</b> When {@link frc.robot.Constants#INFINITE_HOPPER} is true, this
   * method does nothing - the fuel count never decreases, simulating unlimited ammunition for
   * testing.
   */
  public void removeFuel() {
    // If infinite hopper mode is enabled, don't remove fuel
    if (frc.robot.Constants.INFINITE_HOPPER) {
      return;
    }

    if (fuelCount >= 1) {
      fuelCount--;
      logFuelCount();
    }
  }

  /**
   * Checks if the hopper contains at least one fuel piece.
   *
   * <p>Used by the shooter to determine if there's fuel available to launch.
   *
   * <p><b>Infinite Hopper Mode:</b> When {@link frc.robot.Constants#INFINITE_HOPPER} is true, this
   * method always returns true, allowing unlimited shooting for testing purposes.
   *
   * @return true if fuelCount > 0 (or if INFINITE_HOPPER is enabled), false if the hopper is empty
   */
  public boolean hasFuel() {
    // If infinite hopper mode is enabled, always have fuel
    if (frc.robot.Constants.INFINITE_HOPPER) {
      return true;
    }

    return fuelCount > 0;
  }

  /**
   * Clears all fuel from the hopper, setting the count to zero.
   *
   * <p>Used when resetting the simulation or at the start of a new match/test.
   */
  public void reset() {
    fuelCount = 0;
    logFuelCount();
  }

  /**
   * Logs the current fuel count to AdvantageKit.
   *
   * <p>This is called automatically whenever the fuel count changes, allowing you to see the hopper
   * state in AdvantageScope's real-time view.
   */
  private void logFuelCount() {
    Logger.recordOutput("Simulation/VirtualHopper/FuelCount", fuelCount);
  }
}
