package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import java.util.Objects;

public final class ShootLoad {

  private ShootLoad() {
    // Utility class
  }

  /**
   * Runs the launcher for a fixed duration using the subsystem's
   * built-in launch behavior (auto-velocity, limelight, dashboard).
   */
  public static Command shootForTime(CANFuelSubsystem shooter, double seconds) {
    Objects.requireNonNull(shooter, "Shooter subsystem cannot be null");

    if (seconds <= 0.0) {
      throw new IllegalArgumentException("Shoot time must be > 0 seconds");
    }

    return shooter
        .launchCommand()
        .withTimeout(seconds)
        .withName("ShootPreload(" + seconds + "s)");
  }
}
