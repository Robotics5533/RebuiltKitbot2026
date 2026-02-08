package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.AllianceUtil;

public class AutoAlignHub extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;
  private final CommandXboxController controller;
  private final SwerveRequest.FieldCentric driveRequest;

  
  private final ProfiledPIDController alignPID = new ProfiledPIDController(
      Constants.DriveConstants.ALIGN_PID_P, 
      Constants.DriveConstants.ALIGN_PID_I, 
      Constants.DriveConstants.ALIGN_PID_D,
      new TrapezoidProfile.Constraints(
          Constants.DriveConstants.ALIGN_MAX_VELOCITY_DEG_PER_SEC, 
          Constants.DriveConstants.ALIGN_MAX_ACCEL_DEG_PER_SEC_SQ)
  );
  
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  
  private double distanceToTarget = Double.NaN;
  private boolean finishAtSetpoint = false;

  public AutoAlignHub(
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelight,
      CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.controller = controller;
    this.driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    addRequirements(drivetrain); 
    
    
    SmartDashboard.putNumber("AutoAlign/kP", Constants.DriveConstants.ALIGN_PID_P);
    SmartDashboard.putNumber("AutoAlign/kI", Constants.DriveConstants.ALIGN_PID_I);
    SmartDashboard.putNumber("AutoAlign/kD", Constants.DriveConstants.ALIGN_PID_D);
    SmartDashboard.putNumber("AutoAlign/kS", Constants.DriveConstants.ALIGN_KS);
    
    
    SmartDashboard.setDefaultBoolean("AutoAlign/TestMode", true);
  }

  public AutoAlignHub finishWhenAligned() {
    this.finishAtSetpoint = true;
    return this;
  }

  @Override
  public void initialize() {
    
    double kP = SmartDashboard.getNumber("AutoAlign/kP", Constants.DriveConstants.ALIGN_PID_P);
    double kI = SmartDashboard.getNumber("AutoAlign/kI", Constants.DriveConstants.ALIGN_PID_I);
    double kD = SmartDashboard.getNumber("AutoAlign/kD", Constants.DriveConstants.ALIGN_PID_D);
    
    alignPID.setPID(kP, kI, kD);
    alignPID.enableContinuousInput(-180, 180);
    
    alignPID.setTolerance(
        Constants.DriveConstants.ALIGN_TOLERANCE_DEG, 
        Constants.DriveConstants.ALIGN_TOLERANCE_VEL_DEG_PER_SEC
    );
    alignPID.reset(drivetrain.getState().Pose.getRotation().getDegrees());
    
    distanceToTarget = Double.NaN;
  }

  @Override
  public void execute() {
    
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d hubPose = AllianceUtil.getHubPose();
    distanceToTarget = robotPose.getTranslation().getDistance(hubPose.getTranslation());
    
    double currentHeading = robotPose.getRotation().getDegrees();
    
    
    double targetAngle;
    boolean testMode = SmartDashboard.getBoolean("AutoAlign/TestMode", true);
    
    if (testMode) {
        targetAngle = 180.0;
    } else {
        
        double dx = hubPose.getX() - robotPose.getX();
        double dy = hubPose.getY() - robotPose.getY();
        targetAngle = new Rotation2d(Math.atan2(dy, dx)).getDegrees();
    }
    
    
    double pidOutput = alignPID.calculate(currentHeading, targetAngle);

    
    
    double kS = SmartDashboard.getNumber("AutoAlign/kS", Constants.DriveConstants.ALIGN_KS);
    
    if (alignPID.atSetpoint()) {
        pidOutput = 0.0;
    } else if (Math.abs(pidOutput) > 0.001) {
        pidOutput += Math.signum(pidOutput) * kS;
    }
    
    
    SmartDashboard.putNumber("AutoAlign/CurrentAngle", currentHeading);
    SmartDashboard.putNumber("AutoAlign/TargetAngle", targetAngle);
    SmartDashboard.putNumber("AutoAlign/Error", alignPID.getPositionError());
    SmartDashboard.putNumber("AutoAlign/Output", pidOutput);
    SmartDashboard.putBoolean("AutoAlign/AtSetpoint", alignPID.atSetpoint());

    
    double vx = controller != null ? -controller.getLeftY() * maxSpeed : 0.0;
    double vy = controller != null ? -controller.getLeftX() * maxSpeed : 0.0;
    
    drivetrain.setControl(driveRequest
        .withVelocityX(vx) 
        .withVelocityY(vy)
        .withRotationalRate(pidOutput * maxAngularRate)); 
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    return finishAtSetpoint && alignPID.atSetpoint();
  }

  public double getDistanceToTarget() {
    return distanceToTarget;
  }
  
  public boolean isAligned() {
    return alignPID.atSetpoint();
  }

  
  public static boolean isAligned(CommandSwerveDrivetrain drivetrain) {
    Pose2d robotPose = drivetrain.getState().Pose;
    
    
    boolean testMode = SmartDashboard.getBoolean("AutoAlign/TestMode", true);
    double targetDegrees;

    if (testMode) {
        targetDegrees = 180.0;
    } else {
        Pose2d hubPose = AllianceUtil.getHubPose();
        double dx = hubPose.getX() - robotPose.getX();
        double dy = hubPose.getY() - robotPose.getY();
        Rotation2d angleToHub = new Rotation2d(Math.atan2(dy, dx));
        targetDegrees = angleToHub.getDegrees();
    }
    
    double currentDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
    
    double error = MathUtil.inputModulus(targetDegrees - currentDegrees, -180, 180);
    return Math.abs(error) < Constants.DriveConstants.ALIGN_TOLERANCE_DEG;
  }
}