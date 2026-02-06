package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax feederMotor;
  private final SparkMax launcherMotor;
  private final CommandSwerveDrivetrain drivetrain;
  
  private final SimpleMotorFeedforward launcherFF;
  private final SimpleMotorFeedforward feederFF;

  private Double overrideDistanceMeters = null;
  private boolean useAutoVelocity = true;
  private double targetLauncherRPM = 0;
  private double targetFeederRPM = 0;
  private boolean intakeActive = false;
  private final SysIdRoutine launcherSysId;
  private final SysIdRoutine feederSysId;

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    feederMotor = createFeederMotor();
    launcherMotor = createLauncherMotor();
    
    launcherFF = new SimpleMotorFeedforward(Constants.ShooterConstants.LAUNCHER_kS, Constants.ShooterConstants.LAUNCHER_kV, Constants.ShooterConstants.LAUNCHER_kA);
    feederFF = new SimpleMotorFeedforward(Constants.ShooterConstants.FEEDER_kS, Constants.ShooterConstants.FEEDER_kV, Constants.ShooterConstants.FEEDER_kA);

    SmartDashboard.putBoolean("Shooter/Use Auto Velocity", true);
    SmartDashboard.putNumber("Shooter/Manual Launcher RPM", Constants.ShooterConstants.MANUAL_LAUNCHER_RPM);

    launcherSysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            v -> launcherMotor.setVoltage(v.in(Volts)),
            log -> {
              log.motor("launcher")
                  .voltage(Volts.of(launcherMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                  .angularPosition(Rotations.of(launcherMotor.getEncoder().getPosition()))
                  .angularVelocity(RotationsPerSecond.of(launcherMotor.getEncoder().getVelocity() / 60.0));
            },
            this,
            "ShooterLauncher"));
    feederSysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            v -> feederMotor.setVoltage(v.in(Volts)),
            log -> {
              log.motor("feeder")
                  .voltage(Volts.of(feederMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                  .angularPosition(Rotations.of(feederMotor.getEncoder().getPosition()))
                  .angularVelocity(RotationsPerSecond.of(feederMotor.getEncoder().getVelocity() / 60.0));
            },
            this,
            "ShooterFeeder"));
  }

  private SparkMax createFeederMotor() {
    var motor = new SparkMax(Constants.ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    var config = new SparkMaxConfig();
    
    // RIO-side control, so no SparkMax PID/FF config needed here
    config.smartCurrentLimit(Constants.ShooterConstants.FEEDER_CURRENT_LIMIT);
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private SparkMax createLauncherMotor() {
    var motor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    var config = new SparkMaxConfig();
    
    config.inverted(true);
    
    // RIO-side control, so no SparkMax PID/FF config needed here
    config.smartCurrentLimit(Constants.ShooterConstants.LAUNCHER_CURRENT_LIMIT);
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  @Override
  public void periodic() {
    useAutoVelocity = SmartDashboard.getBoolean("Shooter/Use Auto Velocity", true);
    
    double currentLauncherRPM = launcherMotor.getEncoder().getVelocity();
    double currentFeederRPM = feederMotor.getEncoder().getVelocity();

    SmartDashboard.putNumber("Shooter/Launcher RPM", currentLauncherRPM);
    SmartDashboard.putNumber("Shooter/Feeder RPM", currentFeederRPM);
    SmartDashboard.putNumber("Shooter/Target Launcher RPM", targetLauncherRPM);
    SmartDashboard.putNumber("Shooter/Target Feeder RPM", targetFeederRPM);
    SmartDashboard.putBoolean("Shooter/Launcher At Speed", isLauncherAtSpeed());

    // Voltage Control Loop
    if (Math.abs(targetLauncherRPM) < 1.0) {
      launcherMotor.setVoltage(0);
    } else {
      double launcherVolts = launcherFF.calculate(targetLauncherRPM);
      launcherMotor.setVoltage(launcherVolts);
    }

    if (Math.abs(targetFeederRPM) < 1.0) {
      feederMotor.setVoltage(0);
    } else {
      double feederVolts = feederFF.calculate(targetFeederRPM);
      feederMotor.setVoltage(feederVolts);
    }
  }

  private void setVelocities(double feederRPM, double launcherRPM) {
    targetFeederRPM = feederRPM;
    targetLauncherRPM = launcherRPM;
  }

  public void intake() {
    setVelocities(Constants.ShooterConstants.INTAKING_FEEDER_RPM, Constants.ShooterConstants.INTAKING_LAUNCHER_RPM);
    intakeActive = true;
  }

  public void spinUp() {
    double launcherRPM = getLauncherVelocity();
    setVelocities(Constants.ShooterConstants.IDLE_FEEDER_RPM, launcherRPM);
  }

  public void launch() {
    double launcherRPM = getLauncherVelocity();
    setVelocities(Constants.ShooterConstants.INTAKING_FEEDER_RPM, launcherRPM);
  }

  public void stop() {
    setVelocities(0, 0);
    intakeActive = false;
  }

  public void setTargetDistance(Double distanceMeters) {
    overrideDistanceMeters = distanceMeters;
    SmartDashboard.putNumber("Shooter/Override Distance", distanceMeters != null ? distanceMeters : Double.NaN);
  }

  public boolean isLauncherAtSpeed() {
    double currentRPM = launcherMotor.getEncoder().getVelocity();
    return Math.abs(currentRPM - targetLauncherRPM) < Constants.ShooterConstants.VELOCITY_TOLERANCE_RPM;
  }

  private double getLauncherVelocity() {
    if (useAutoVelocity) {
      double distance = getDistanceToTarget();
      
      if (!Double.isNaN(distance)) {
        return Constants.ShooterConstants.FuelVelocity.get(distance);
      }
    }
    return SmartDashboard.getNumber("Shooter/Manual Launcher RPM", Constants.ShooterConstants.MANUAL_LAUNCHER_RPM);
  }

  private double getDistanceToTarget() {
    if (overrideDistanceMeters != null && !overrideDistanceMeters.isNaN()) {
      return overrideDistanceMeters;
    }
    double llDist = getDistanceFromLimelight();
    if (!Double.isNaN(llDist)) {
      return llDist;
    }
    var robotPose = drivetrain.getState().Pose;
    var hubPose = frc.robot.utils.AllianceUtil.getHubPose();
    return robotPose.getTranslation().getDistance(hubPose.getTranslation());
  }

  private double getDistanceFromLimelight() {
    var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightConstants.LIMELIGHT_NAME);
    if (estimate.tagCount > 0) {
      return estimate.pose.getTranslation().getDistance(frc.robot.utils.AllianceUtil.getHubPose().getTranslation());
    }
    return Double.NaN;
  }

  public Command intakeCommand() {
    return runEnd(this::intake, this::stop).withName("Intake");
  }

  public Command spinUpCommand() {
    return run(this::spinUp).withName("SpinUp");
  }

  public Command launchCommand() {
    return run(this::launch).finallyDo(this::stop).withName("Launch");
  }

  public Command spinUpAndShootCommand() {
    return Commands.sequence(
        run(this::spinUp).until(this::isLauncherAtSpeed),
        run(this::launch)
    ).finallyDo(this::stop).withName("SpinUpAndShoot");
  }

  public boolean isIntaking() {
    return intakeActive;
  }

  public Command sysIdLauncherQuasistaticForward() {
    return launcherSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdLauncherQuasistaticReverse() {
    return launcherSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdLauncherDynamicForward() {
    return launcherSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdLauncherDynamicReverse() {
    return launcherSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFeederQuasistaticForward() {
    return feederSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFeederQuasistaticReverse() {
    return feederSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFeederDynamicForward() {
    return feederSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFeederDynamicReverse() {
    return feederSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }
}
