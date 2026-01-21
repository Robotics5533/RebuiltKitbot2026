package frc.robot.subsystems;
import java.util.Set;

public class Constants {
  public static final Set<Integer> RED_HUB_TAGS = Set.of(2, 10, 5);

  public static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 27, 26, 25, 21, 24);

  public static final String LIMELIGHT_NAME = "limelight";

  public static final double KP = 0.035;
  public static final double KI = 0.0;
  public static final double KD = 0.002;

  public static final double MAX_OMEGA = 3.0;
  public static final double SLEW_RATE = 8.0;

  public static final double TX_DEADBAND_DEG = 0.5;
  public static final double PID_TOLERANCE_DEG = 1.0;
}
