package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class CANFuelSubsystem extends SubsystemBase {

  private static final int INTAKE_LAUNCHER_MOTOR_ID = 15;
  private static final int FEEDER_MOTOR_ID = 14;

  public static final double INTAKING_FEEDER_VOLTAGE = 12.0;
  public static final double INTAKING_INTAKE_VOLTAGE = -10.0;

  public static final double SPIN_UP_FEEDER_VOLTAGE = 6.0;
  public static final double LAUNCHING_FEEDER_VOLTAGE = 9.0;
  public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;

  public static final double SPIN_UP_SECONDS = 1.0;

  private static final String KEY_AUTO_VELOCITY = "Fuel/Use Auto Velocity";

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;

  public CANFuelSubsystem() {
    feederRoller = createFeederMotor();
    intakeLauncherRoller = createLauncherMotor();
    SmartDashboard.putBoolean(KEY_AUTO_VELOCITY, true);
  }

  private SparkMax createFeederMotor() {
    SparkMax motor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private SparkMax createLauncherMotor() {
    SparkMax motor = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private void setVoltages(double feederVolts, double launcherVolts) {
    feederRoller.setVoltage(feederVolts);
    intakeLauncherRoller.setVoltage(launcherVolts);
    SmartDashboard.putNumber("Fuel/Feeder Voltage Cmd", feederVolts);
    SmartDashboard.putNumber("Fuel/Launcher Voltage Cmd", launcherVolts);
  }

  public void intake() {
    setVoltages(INTAKING_FEEDER_VOLTAGE, INTAKING_INTAKE_VOLTAGE);
  }

  public void spinUp() {
    setVoltages(SPIN_UP_FEEDER_VOLTAGE, LAUNCHING_LAUNCHER_VOLTAGE);
  }

  public void launch() {
    setVoltages(20, 20);
  }

  public void stop() {
    setVoltages(0.0, 0.0);
  }

  private double getLauncherVoltage() {
    if (SmartDashboard.getBoolean(KEY_AUTO_VELOCITY, true)) {
      double distance = getDistanceToValidTag();
      if (!Double.isNaN(distance)) {
        return getLauncherVoltageFromDistance(distance);
      }
    }
    return LAUNCHING_LAUNCHER_VOLTAGE;
  }

  private double getDistanceToValidTag() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Constants.LIMELIGHT_NAME);
    SmartDashboard.putNumber("Fuel/Fiducial Count", fiducials.length);

    for (RawFiducial fid : fiducials) {
      // if (Constants.RED_HUB_TAGS.contains(fid.id)
      // || Constants.BLUE_HUB_TAGS.contains(fid.id)) {

      SmartDashboard.putNumber("Fuel/Target Tag ID", fid.id);
      SmartDashboard.putNumber("Fuel/Target Distance M", fid.distToRobot);

      return fid.distToRobot;
      // }
    }

    SmartDashboard.putNumber("Fuel/Target Tag ID", -1);
    SmartDashboard.putNumber("Fuel/Target Distance M", Double.NaN);

    return Double.NaN;
  }

  private double getLauncherVoltageFromDistance(double distanceMeters) {
    double a = 0.2;
    double b = 4.2;
    double c = LAUNCHING_LAUNCHER_VOLTAGE;

    return MathUtil.clamp(
        a * distanceMeters * distanceMeters + b * distanceMeters + c,
        0.0,
        12.0);
  }

  public Command intakeCommand() {
    return runEnd(this::intake, this::stop);
  }

  public Command spinUpCommand() {
    return run(this::spinUp);
  }

  public Command launchCommand() {
    return run(this::launch)
        .finallyDo(this::stop);
  }
}
