package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class CANFuelSubsystem extends SubsystemBase {

  private static final int INTAKE_LAUNCHER_MOTOR_ID = 10;
  private static final int FEEDER_MOTOR_ID = 11;

  private static final double INTAKING_FEEDER_VOLTAGE = 6.0;
  private static final double INTAKING_INTAKE_VOLTAGE = 6.0;
  private static final double LAUNCHING_FEEDER_VOLTAGE = 8.0;
  private static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.0;
  private static final double SPIN_UP_FEEDER_VOLTAGE = -2.0;

  private static final int FEEDER_MOTOR_CURRENT_LIMIT = 40;
  private static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 40;

  private static final String KEY_INTAKE_FEEDER = "Intaking feeder roller value";
  private static final String KEY_INTAKE_LAUNCHER = "Intaking intake roller value";
  private static final String KEY_LAUNCH_FEEDER = "Launching feeder roller value";
  private static final String KEY_LAUNCH_LAUNCHER = "Launching launcher roller value";
  private static final String KEY_SPINUP_FEEDER = "Spin-up feeder roller value";
  private static final String KEY_AUTO_VELOCITY = "Use Auto Velocity";

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;

  private FlywheelSim launcherSim;
  private boolean simHasBall = true;
  private boolean simShotLatched = false;

  public CANFuelSubsystem() {
    intakeLauncherRoller = createLauncherMotor();
    feederRoller = createFeederMotor();

    SmartDashboard.putNumber(KEY_INTAKE_FEEDER, INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(KEY_INTAKE_LAUNCHER, INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(KEY_LAUNCH_FEEDER, LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(KEY_LAUNCH_LAUNCHER, LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber(KEY_SPINUP_FEEDER, SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putBoolean(KEY_AUTO_VELOCITY, true);

    if (RobotBase.isSimulation()) {
      launcherSim =
    new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1),
            0.025,   
            1.0      
        ),
        DCMotor.getNEO(1),
        1.0
    );
      SmartDashboard.putNumber("Sim Distance To Hub", 3.0);
      SmartDashboard.putBoolean("Sim Has Ball", true);
      SmartDashboard.putBoolean("Shot Fired (Sim)", false);
      SmartDashboard.putBoolean("Shot Scored (Sim)", false);
    }
  }

  private SparkMax createFeederMotor() {
    SparkMax motor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private SparkMax createLauncherMotor() {
    SparkMax motor = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  public void intake() {
    feederRoller.setVoltage(
        SmartDashboard.getNumber(KEY_INTAKE_FEEDER, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(
        SmartDashboard.getNumber(KEY_INTAKE_LAUNCHER, INTAKING_INTAKE_VOLTAGE));
  }

  public void eject() {
    feederRoller.setVoltage(
        -SmartDashboard.getNumber(KEY_INTAKE_FEEDER, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(
        -SmartDashboard.getNumber(KEY_INTAKE_LAUNCHER, INTAKING_INTAKE_VOLTAGE));
  }

  public void launch() {
    feederRoller.setVoltage(
        SmartDashboard.getNumber(KEY_LAUNCH_FEEDER, LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(getLauncherVoltage());
  }

  public void spinUp() {
    feederRoller.setVoltage(
        SmartDashboard.getNumber(KEY_SPINUP_FEEDER, SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(getLauncherVoltage());
  }

  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  private double getLauncherVoltage() {
    if (SmartDashboard.getBoolean(KEY_AUTO_VELOCITY, true)) {
      double dist = getDistanceToValidTag();
      if (!Double.isNaN(dist)) {
        return getLauncherVoltageFromDistance(dist);
      }
    }
    return SmartDashboard.getNumber(KEY_LAUNCH_LAUNCHER, LAUNCHING_LAUNCHER_VOLTAGE);
  }

  private double getDistanceToValidTag() {
    if (RobotBase.isSimulation()) {
      return SmartDashboard.getNumber("Sim Distance To Hub", Double.NaN);
    }

    RawFiducial[] fiducials =
        LimelightHelpers.getRawFiducials(Constants.LIMELIGHT_NAME);

    for (RawFiducial fid : fiducials) {
      if (Constants.RED_HUB_TAGS.contains(fid.id)
          || Constants.BLUE_HUB_TAGS.contains(fid.id)) {
        return fid.distToRobot;
      }
    }
    return Double.NaN;
  }

  private double getLauncherVoltageFromDistance(double distanceMeters) {
    if (distanceMeters < 1.0) {
      return 9.0;
    }
    if (distanceMeters > 5.0) {
      return 12.0;
    }
    return 9.0 + (distanceMeters - 1.0) * (3.0 / 4.0);
  }

  @Override
  public void simulationPeriodic() {
    launcherSim.setInputVoltage(
        intakeLauncherRoller.getAppliedOutput() * 12.0);
    launcherSim.update(0.02);

    double rpm = launcherSim.getAngularVelocityRPM();
    SmartDashboard.putNumber("Shooter RPM (Sim)", rpm);

    simHasBall = SmartDashboard.getBoolean("Sim Has Ball", simHasBall);

    boolean feederActive = feederRoller.getAppliedOutput() > 0.2;

    if (feederActive && !simShotLatched && simHasBall) {
      simShotLatched = true;
      evaluateSimShot(rpm);
    }

    if (!feederActive) {
      simShotLatched = false;
    }
  }

  private void evaluateSimShot(double rpm) {
    double distance = getDistanceToValidTag();
    double actualVoltage = getLauncherVoltage();
    double expectedVoltage = getLauncherVoltageFromDistance(distance);

    boolean rpmReady = rpm >= 2500.0;
    boolean voltageMatch = Math.abs(actualVoltage - expectedVoltage) <= 0.75;
    boolean scored = rpmReady && voltageMatch;

    SmartDashboard.putBoolean("Shot Fired (Sim)", true);
    SmartDashboard.putBoolean("Shot Scored (Sim)", scored);
    SmartDashboard.putNumber("Shot Distance (Sim)", distance);
    SmartDashboard.putNumber("Expected Voltage (Sim)", expectedVoltage);

    simHasBall = false;
    SmartDashboard.putBoolean("Sim Has Ball", false);
  }

  public Command intakeCommand() {
    return runEnd(this::intake, this::stop);
  }

  public Command ejectCommand() {
    return runEnd(this::eject, this::stop);
  }

  public Command launchCommand() {
    return runEnd(this::launch, this::stop);
  }

  public Command spinUpCommand() {
    return runEnd(this::spinUp, this::stop);
  }
}
