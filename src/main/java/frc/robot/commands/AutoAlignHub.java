package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceUtil;

public class AutoAlignHub extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController controller;
  private final SwerveRequest.FieldCentric driveRequest;

  private final PIDController alignPID = new PIDController(
      Constants.DriveConstants.ALIGN_PID_P, 
      Constants.DriveConstants.ALIGN_PID_I, 
      Constants.DriveConstants.ALIGN_PID_D);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.DriveConstants.ALIGN_ROTATION_LIMIT);
  
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  
  private double distanceToTarget = Double.NaN;
  private String limelightName;

  private boolean finishAtSetpoint = false;

  public AutoAlignHub(
      CommandSwerveDrivetrain drivetrain,
      CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.limelightName = Constants.LimelightConstants.LIMELIGHT_NAME;
    this.driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    addRequirements(drivetrain);
  }

  public AutoAlignHub finishWhenAligned() {
    this.finishAtSetpoint = true;
    return this;
  }

  @Override
  public void initialize() {
    alignPID.reset();
    alignPID.enableContinuousInput(-180, 180);
    alignPID.setTolerance(Constants.DriveConstants.ALIGN_TOLERANCE_DEG);
    rotationLimiter.reset(0);
    distanceToTarget = Double.NaN;
  }

  @Override
  public void execute() {
    distanceToTarget = AllianceUtil.getDistanceToHub(drivetrain, limelightName);
    double targetAngle = AllianceUtil.getTargetHeadingToHub(drivetrain, limelightName);

    double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
    double pidOutput = alignPID.calculate(currentHeading, targetAngle);
    
    double rotationRate = rotationLimiter.calculate(pidOutput * maxAngularRate);
    rotationRate = MathUtil.clamp(rotationRate, -maxAngularRate, maxAngularRate);

    drivetrain.setControl(driveRequest
        .withVelocityX(-controller.getLeftY() * maxSpeed)
        .withVelocityY(-controller.getLeftX() * maxSpeed)
        .withRotationalRate(rotationRate));

    SmartDashboard.putNumber("AutoAlign/TargetAngle", targetAngle);
    SmartDashboard.putNumber("AutoAlign/CurrentAngle", currentHeading);
    SmartDashboard.putNumber("AutoAlign/Distance", distanceToTarget);
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
    return AllianceUtil.getDistanceToHub(drivetrain, limelightName);
  }

  public double getLimelightDistance() {
    var estimate = frc.robot.utils.LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    if (estimate.tagCount > 0) {
      return estimate.pose.getTranslation().getDistance(AllianceUtil.getHubPose().getTranslation());
    }
    return Double.NaN;
  }

  public boolean isAligned() {
    return alignPID.atSetpoint();
  }

  public static boolean isAligned(CommandSwerveDrivetrain drivetrain) {
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d hubPose = AllianceUtil.getHubPose();
    
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();
    
    Rotation2d angleToHub = new Rotation2d(Math.atan2(dy, dx));
    double targetDegrees = angleToHub.getDegrees();
    double currentDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
    
    double error = MathUtil.inputModulus(targetDegrees - currentDegrees, -180, 180);
    return Math.abs(error) < 2.0;
  }
}
