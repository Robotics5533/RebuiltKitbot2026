package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class CANFuelSubsystem extends SubsystemBase {

  private static final int FEEDER_MOTOR_ID = 14;
  private static final int LAUNCHER_MOTOR_ID = 15;

  // Velocity setpoints in RPM
  private static final double INTAKING_FEEDER_RPM = 3000;
  private static final double INTAKING_LAUNCHER_RPM = -2500;
  private static final double IDLE_FEEDER_RPM = 1500;
  
  // PID Constants for velocity control
  private static final double LAUNCHER_kP = 0.0002;
  private static final double LAUNCHER_kI = 0.0;
  private static final double LAUNCHER_kD = 0.0;
  private static final double LAUNCHER_kFF = 0.000175; // ~1/5700 for NEO 550
  
  private static final double FEEDER_kP = 0.0001;
  private static final double FEEDER_kI = 0.0;
  private static final double FEEDER_kD = 0.0;
  private static final double FEEDER_kFF = 0.000175;

  // Tolerance for velocity control (RPM)
  private static final double VELOCITY_TOLERANCE_RPM = 100;

  private final SparkMax feederMotor;
  private final SparkMax launcherMotor;
  
  // State tracking
  private Double overrideDistanceMeters = null;
  private boolean useAutoVelocity = true;
  private double targetLauncherRPM = 0;
  private double targetFeederRPM = 0;

  public CANFuelSubsystem() {
    feederMotor = createFeederMotor();
    launcherMotor = createLauncherMotor();
    
    // Initialize dashboard controls
    SmartDashboard.putBoolean("Fuel/Use Auto Velocity", true);
    SmartDashboard.putNumber("Fuel/Manual Launcher RPM", 4000);
  }

  private SparkMax createFeederMotor() {
    SparkMax motor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    
    // Configure PID for velocity control
    config.closedLoop
       
        .pid(FEEDER_kP, FEEDER_kI, FEEDER_kD)
        .velocityFF(FEEDER_kFF);
    
    // Current limiting for thermal protection
    config.smartCurrentLimit(30);
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private SparkMax createLauncherMotor() {
    SparkMax motor = new SparkMax(LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    
    config.inverted(true);
    
    // Configure PID for velocity control
    config.closedLoop

        .pid(LAUNCHER_kP, LAUNCHER_kI, LAUNCHER_kD)
        .velocityFF(LAUNCHER_kFF);
    
    // Current limiting for thermal protection
    config.smartCurrentLimit(40);
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  @Override
  public void periodic() {
    // Update auto velocity setting from dashboard
    useAutoVelocity = SmartDashboard.getBoolean("Fuel/Use Auto Velocity", true);
    
    // Telemetry
    SmartDashboard.putNumber("Fuel/Launcher RPM", launcherMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Fuel/Feeder RPM", feederMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Fuel/Target Launcher RPM", targetLauncherRPM);
    SmartDashboard.putNumber("Fuel/Target Feeder RPM", targetFeederRPM);
    SmartDashboard.putNumber("Fuel/Launcher Current", launcherMotor.getOutputCurrent());
    SmartDashboard.putNumber("Fuel/Feeder Current", feederMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Fuel/Launcher At Speed", isLauncherAtSpeed());
  }

  /**
   * Set target velocities for both motors
   */
  private void setVelocities(double feederRPM, double launcherRPM) {
    targetFeederRPM = feederRPM;
    targetLauncherRPM = launcherRPM;
    
    feederMotor.getClosedLoopController().setReference(
        feederRPM, 
        ControlType.kVelocity);
    launcherMotor.getClosedLoopController().setReference(
        launcherRPM, 
        ControlType.kVelocity);
  }

  /**
   * Run intake - pulls note in
   */
  public void intake() {
    setVelocities(INTAKING_FEEDER_RPM, INTAKING_LAUNCHER_RPM);
  }

  /**
   * Spin up launcher to target velocity based on distance
   */
  public void spinUp() {
    double launcherRPM = getLauncherVelocity();
    setVelocities(IDLE_FEEDER_RPM, launcherRPM);
  }

  /**
   * Launch note - feeds into spinning launcher
   */
  public void launch() {
    double launcherRPM = getLauncherVelocity();
    // Use higher feeder speed during launch to push note through
    setVelocities(INTAKING_FEEDER_RPM, launcherRPM);
  }

  /**
   * Stop all motors
   */
  public void stop() {
    setVelocities(0, 0);
  }

  /**
   * Set the distance to target from an external source (like alignment command)
   * This takes priority over Limelight distance calculation
   * @param distanceMeters distance to target in meters, or null to use Limelight
   */
  public void setTargetDistance(Double distanceMeters) {
    overrideDistanceMeters = distanceMeters;
    SmartDashboard.putNumber("Fuel/Override Distance", 
        distanceMeters != null ? distanceMeters : Double.NaN);
  }

  /**
   * Check if launcher is at target velocity
   */
  public boolean isLauncherAtSpeed() {
    double currentRPM = launcherMotor.getEncoder().getVelocity();
    return Math.abs(currentRPM - targetLauncherRPM) < VELOCITY_TOLERANCE_RPM;
  }

  /**
   * Get launcher velocity based on distance using interpolation map from Constants
   */
  private double getLauncherVelocity() {
    if (useAutoVelocity) {
      double distance = getDistanceToTarget();
      
      if (!Double.isNaN(distance)) {
        double velocity = Constants.RobotConstants.FuelVelocity.get(distance);
        SmartDashboard.putString("Fuel/Velocity Source", "Interpolation");
        SmartDashboard.putNumber("Fuel/Calculated Velocity RPM", velocity);
        return velocity;
      }
    }
    
    // Fallback to manual velocity from dashboard
    SmartDashboard.putString("Fuel/Velocity Source", "Manual Dashboard");
    return SmartDashboard.getNumber("Fuel/Manual Launcher RPM", 4000);
  }

  /**
   * Get distance to target with priority system
   */
  private double getDistanceToTarget() {
    // First priority: use override distance from alignment command
    if (overrideDistanceMeters != null && !overrideDistanceMeters.isNaN()) {
      SmartDashboard.putString("Fuel/Distance Source", "Alignment Command");
      SmartDashboard.putNumber("Fuel/Distance M", overrideDistanceMeters);
      return overrideDistanceMeters;
    }
    
    // Second priority: get distance from Limelight
    double distance = getDistanceFromLimelight();
    if (!Double.isNaN(distance)) {
      SmartDashboard.putString("Fuel/Distance Source", "Limelight");
      SmartDashboard.putNumber("Fuel/Distance M", distance);
      return distance;
    }
    
    SmartDashboard.putString("Fuel/Distance Source", "None");
    SmartDashboard.putNumber("Fuel/Distance M", Double.NaN);
    return Double.NaN;
  }

  /**
   * Get distance from Limelight to valid tag
   */
  private double getDistanceFromLimelight() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(
        Constants.LimelightConstants.LIMELIGHT_NAME);
    
    SmartDashboard.putNumber("Fuel/Fiducial Count", fiducials.length);

    if (fiducials.length > 0) {
      RawFiducial target = fiducials[0];
      SmartDashboard.putNumber("Fuel/Target Tag ID", target.id);
      return target.distToRobot;
    }

    SmartDashboard.putNumber("Fuel/Target Tag ID", -1);
    return Double.NaN;
  }

  // ==================== COMMANDS ====================

  /**
   * Command to run intake
   */
  public Command intakeCommand() {
    return runEnd(this::intake, this::stop)
        .withName("Fuel: Intake");
  }

  /**
   * Command to spin up launcher
   */
  public Command spinUpCommand() {
    return run(this::spinUp)
        .withName("Fuel: Spin Up");
  }

  /**
   * Command to launch note
   */
  public Command launchCommand() {
    return run(this::launch)
        .finallyDo(this::stop)
        .withName("Fuel: Launch");
  }

  /**
   * Command to spin up and wait until at speed, then launch
   */
  public Command spinUpAndLaunchCommand() {
    return Commands.sequence(
        runOnce(this::spinUp).withName("Start Spin Up"),
        Commands.waitUntil(this::isLauncherAtSpeed).withName("Wait for Speed"),
        run(this::launch).withName("Launch")
    ).finallyDo(this::stop)
     .withName("Fuel: Spin Up and Launch");
  }
}