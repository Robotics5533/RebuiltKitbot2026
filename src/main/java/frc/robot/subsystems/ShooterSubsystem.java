package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

import static edu.wpi.first.units.Units.*;

/**
 * ShooterSubsystem controls the REV ION motors for fuel intake and launching.
 * 
 * This subsystem includes SysId characterization routines for both the launcher and feeder motors.
 * To use SysId:
 * 
 * 1. Add to your RobotContainer or Controls class:
 *    - joystick.button().whileTrue(shooter.launcherSysIdCommand(Direction.kForward));
 *    - joystick.button().whileTrue(feederSysIdCommand(Direction.kReverse));
 *    - joystick.button().whileTrue(shooter.launcherSysIdQuasistaticCommand(Direction.kForward));
 *    - joystick.button().whileTrue(shooter.feederSysIdQuasistaticCommand(Direction.kReverse));
 * 
 * 2. Run each test once (forward and reverse for both dynamic and quasistatic)
 * 3. Export log file and run SysId GUI tool to generate feedforward constants
 * 4. Update Constants.ShooterConstants with generated kS, kV, kA values
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax feederMotor;
  private final SparkMax launcherMotor;
  private final CommandSwerveDrivetrain drivetrain;
  
  private Double overrideDistanceMeters = null;
  private boolean useAutoVelocity = true;
  private double targetLauncherRPM = 0;
  private double targetFeederRPM = 0;

  // SysId Routine instances (initialized in constructor after motors are created)
  private SysIdRoutine launcherSysId;
  private SysIdRoutine feederSysId;

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    feederMotor = createFeederMotor();
    launcherMotor = createLauncherMotor();
    
    // Initialize SysId routines for launcher motor characterization
    // Uses null for log consumer since URCL automatically logs all Spark Max data
    launcherSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            state -> SmartDashboard.putString("Launcher SysId State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> setLauncherVoltage(output.in(Volts)),
            null,  // No log consumer - URCL handles all logging
            this));

    // Initialize SysId routines for feeder motor characterization
    // Uses null for log consumer since URCL automatically logs all Spark Max data
    feederSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            state -> SmartDashboard.putString("Feeder SysId State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> setFeederVoltage(output.in(Volts)),
            null,  // No log consumer - URCL handles all logging
            this));
    
    SmartDashboard.putBoolean("Shooter/Use Auto Velocity", true);
    SmartDashboard.putNumber("Shooter/Manual Launcher RPM", Constants.ShooterConstants.MANUAL_LAUNCHER_RPM);
  }

  private SparkMax createFeederMotor() {
    var motor = new SparkMax(Constants.ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    var config = new SparkMaxConfig();
    
    config.closedLoop
        .pid(Constants.ShooterConstants.FEEDER_kP, Constants.ShooterConstants.FEEDER_kI, Constants.ShooterConstants.FEEDER_kD, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedForward.kV(Constants.ShooterConstants.FEEDER_kFF);
    
    config.smartCurrentLimit(Constants.ShooterConstants.FEEDER_CURRENT_LIMIT);
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private SparkMax createLauncherMotor() {
    var motor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    var config = new SparkMaxConfig();
    
    config.inverted(true);
    
    config.closedLoop
        .pid(Constants.ShooterConstants.LAUNCHER_kP, Constants.ShooterConstants.LAUNCHER_kI, Constants.ShooterConstants.LAUNCHER_kD, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedForward.kV(Constants.ShooterConstants.LAUNCHER_kFF);
    
    config.smartCurrentLimit(Constants.ShooterConstants.LAUNCHER_CURRENT_LIMIT);
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  @Override
  public void periodic() {
    useAutoVelocity = SmartDashboard.getBoolean("Shooter/Use Auto Velocity", true);
    
    SmartDashboard.putNumber("Shooter/Launcher RPM", launcherMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Feeder RPM", feederMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Target Launcher RPM", targetLauncherRPM);
    SmartDashboard.putNumber("Shooter/Target Feeder RPM", targetFeederRPM);
    SmartDashboard.putBoolean("Shooter/Launcher At Speed", isLauncherAtSpeed());
  }

  private void setVelocities(double feederRPM, double launcherRPM) {
    targetFeederRPM = feederRPM;
    targetLauncherRPM = launcherRPM;
    
    feederMotor.getClosedLoopController().setSetpoint(feederRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0);
    launcherMotor.getClosedLoopController().setSetpoint(launcherRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0);
  }

  public void intake() {
    setVelocities(Constants.ShooterConstants.INTAKING_FEEDER_RPM, Constants.ShooterConstants.INTAKING_LAUNCHER_RPM);
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

  /**
   * Sets the launcher motor voltage directly for SysId characterization.
   * Disables closed-loop control temporarily.
   * 
   * @param volts The voltage to apply to the launcher motor
   */
  private void setLauncherVoltage(double volts) {
    launcherMotor.setVoltage(volts);
  }

  /**
   * Sets the feeder motor voltage directly for SysId characterization.
   * Disables closed-loop control temporarily.
   * 
   * @param volts The voltage to apply to the feeder motor
   */
  private void setFeederVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }

  /**
   * Returns the launcher motor SysId routine for characterization testing.
   * Use this command to measure launcher motor parameters (kS, kV, kA).
   * 
   * @param direction The direction (Forward or Reverse) to run the SysId test
   * @return The SysId command sequence for the launcher motor
   */
  public Command launcherSysIdCommand(SysIdRoutine.Direction direction) {
    return launcherSysId.dynamic(direction);
  }

  /**
   * Returns the feeder motor SysId routine for characterization testing.
   * Use this command to measure feeder motor parameters (kS, kV, kA).
   * 
   * @param direction The direction (Forward or Reverse) to run the SysId test
   * @return The SysId command sequence for the feeder motor
   */
  public Command feederSysIdCommand(SysIdRoutine.Direction direction) {
    return feederSysId.dynamic(direction);
  }

  /**
   * Returns a quasistatic SysId routine for the launcher motor.
   * This test slowly ramps voltage and provides smoother characterization data.
   * 
   * @param direction The direction (Forward or Reverse) to run the SysId test
   * @return The quasistatic SysId command sequence for the launcher motor
   */
  public Command launcherSysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return launcherSysId.quasistatic(direction);
  }

  /**
   * Returns a quasistatic SysId routine for the feeder motor.
   * This test slowly ramps voltage and provides smoother characterization data.
   * 
   * @param direction The direction (Forward or Reverse) to run the SysId test
   * @return The quasistatic SysId command sequence for the feeder motor
   */
  public Command feederSysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return feederSysId.quasistatic(direction);
  }
}
