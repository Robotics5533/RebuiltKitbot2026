package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;


public class ShooterSubsystem extends SubsystemBase {

  private static final String KEY_INTAKE_FEEDER =
      "Shooter/Intaking Feeder Volts";
  private static final String KEY_INTAKE_LAUNCHER =
      "Shooter/Intaking Launcher Volts";
  private static final String KEY_LAUNCH_FEEDER =
      "Shooter/Launching Feeder Volts";
  private static final String KEY_LAUNCH_LAUNCHER =
      "Shooter/Launching Launcher Volts";
  private static final String KEY_SPINUP_FEEDER =
      "Shooter/Spin-up Feeder Volts";

  private final SparkMax feederMotor;
  private final SparkMax launcherMotor;

  private double currentDistance = 0.0;
  private boolean useDistanceTuning = false;

  public ShooterSubsystem() {
    feederMotor = createFeederMotor();
    launcherMotor = createLauncherMotor();

    SmartDashboard.putNumber(KEY_INTAKE_FEEDER,
                             Constants.ShooterConstants.INTAKING_FEEDER_VOLTS);
    SmartDashboard.putNumber(
        KEY_INTAKE_LAUNCHER,
        Constants.ShooterConstants.INTAKING_LAUNCHER_VOLTS);
    SmartDashboard.putNumber(KEY_LAUNCH_FEEDER,
                             Constants.ShooterConstants.INTAKING_FEEDER_VOLTS);
    SmartDashboard.putNumber(KEY_LAUNCH_LAUNCHER,
                             Constants.ShooterConstants.MANUAL_LAUNCHER_VOLTS);
    SmartDashboard.putNumber(KEY_SPINUP_FEEDER,
                             Constants.ShooterConstants.IDLE_FEEDER_VOLTS);
    SmartDashboard.putBoolean("Shooter/Use Distance Tuning", useDistanceTuning);
  }

  private SparkMax createFeederMotor() {
    var motor = new SparkMax(Constants.ShooterConstants.FEEDER_MOTOR_ID,
                             MotorType.kBrushless);
    var config = new SparkMaxConfig();

    config.smartCurrentLimit(Constants.ShooterConstants.FEEDER_CURRENT_LIMIT);
    config.voltageCompensation(12.0);

    motor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
    return motor;
  }

  private SparkMax createLauncherMotor() {
    var motor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID,
                             MotorType.kBrushless);
    var config = new SparkMaxConfig();

    config.inverted(true);
    config.smartCurrentLimit(Constants.ShooterConstants.LAUNCHER_CURRENT_LIMIT);
    config.voltageCompensation(12.0);

    motor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
    return motor;
  }

  public void setLauncherVoltage(double volts) {
    launcherMotor.setVoltage(volts);
  }

  public void setFeederVoltage(double volts) { feederMotor.setVoltage(volts); }

  public void stop() {
    launcherMotor.stopMotor();
    feederMotor.stopMotor();
  }

  public void intake() {
    setFeederVoltage(SmartDashboard.getNumber(
        KEY_INTAKE_FEEDER, Constants.ShooterConstants.INTAKING_FEEDER_VOLTS));
    setLauncherVoltage(SmartDashboard.getNumber(
        KEY_INTAKE_LAUNCHER,
        Constants.ShooterConstants.INTAKING_LAUNCHER_VOLTS));
  }

  public void launch() {

    setFeederVoltage(SmartDashboard.getNumber(
        KEY_LAUNCH_FEEDER, Constants.ShooterConstants.INTAKING_FEEDER_VOLTS));
    setLauncherVoltage(getTargetVolts());
  }

  public void spinUp() {
    setFeederVoltage(SmartDashboard.getNumber(
        KEY_SPINUP_FEEDER, Constants.ShooterConstants.IDLE_FEEDER_VOLTS));
    setLauncherVoltage(getTargetVolts());
  }

  public double getLauncherVelocity() {
    return launcherMotor.getEncoder().getVelocity();
  }

  public double getLauncherVoltage() {
    return launcherMotor.getAppliedOutput() * launcherMotor.getBusVoltage();
  }

  public Command intakeCommand() {
    return runEnd(this::intake, this::stop).withName("Intake");
  }

  public Command launchCommand() {
    return runEnd(this::launch, this::stop).withName("Launch");
  }

  public Command spinUpCommand(DoubleSupplier voltageSupplier) {
    return runEnd(() -> {
             setLauncherVoltage(voltageSupplier.getAsDouble());
             setFeederVoltage(SmartDashboard.getNumber(
                 KEY_SPINUP_FEEDER,
                 Constants.ShooterConstants.IDLE_FEEDER_VOLTS));
           }, this::stop).withName("SpinUp");
  }

  public Command spinUpCommand() {
    return runEnd(this::spinUp, this::stop).withName("SpinUp");
  }

  public Command spinUpUntilReadyCommand() {
    return run(this::spinUp)
        .until(this::isLauncherAtSpeed)
        .withName("SpinUpUntilReady");
  }

  public Command spinUpAndShootCommand() {
    return spinUpUntilReadyCommand()
        .andThen(run(this::launch))
        .finallyDo(interrupted -> stop())
        .withName("SpinUpAndShoot");
  }

  public Command stopCommand() { return runOnce(this::stop).withName("Stop"); }

  public double getTargetVolts() {
    if (useDistanceTuning) {
      double dist =
          SmartDashboard.getNumber("Shooter/Test Distance", currentDistance);
      return Constants.ShooterConstants.FuelVolts.get(dist);
    }
    return SmartDashboard.getNumber(
        KEY_LAUNCH_LAUNCHER, Constants.ShooterConstants.MANUAL_LAUNCHER_VOLTS);
  }

  public void setTargetDistance(Double distance) {
    if (distance != null) {
      this.currentDistance = distance;

      SmartDashboard.putNumber("Shooter/Test Distance", distance);
    }
  }

  public boolean isLauncherAtSpeed() {
    double target = getTargetVolts();
    if (Math.abs(target) > 2.0) {
      return Math.abs(getLauncherVelocity()) > (Math.abs(target) * 300.0);
    }
    return true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Launcher RPM", getLauncherVelocity());
    SmartDashboard.putNumber("Shooter/Feeder RPM",
                             feederMotor.getEncoder().getVelocity());
    useDistanceTuning =
        SmartDashboard.getBoolean("Shooter/Use Distance Tuning", false);
    SmartDashboard.setDefaultNumber("Shooter/Test Distance", currentDistance);
  }
}