// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceUtil;

public class AlignTargetOdometry extends Command {
  private final CommandXboxController m_controller;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric drive;
  private final String limelightName;

  // PID tuned for fast, stable alignment without overshoot
  // Lower P for less aggressive response, higher D for strong damping
  private final PIDController m_alignPID = new PIDController(0.025, 0.0, 0.008);
  
  // Slew rate limiter to prevent jerky motion
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(4.0); // rad/s^2
  
  private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  
  // Alignment tolerance and deadband
  private static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
  private static final double LIMELIGHT_MIN_AREA = 0.1; // Minimum target area to trust vision
  
  private Pose2d targetPose = new Pose2d();
  private double distanceToTarget = Double.NaN; // Distance in meters for shooter calculations

  public AlignTargetOdometry(
      CommandSwerveDrivetrain drivetrain,
      SwerveRequest.FieldCentric drive,
      CommandXboxController controller,
      boolean feed) {
    this(drivetrain, drive, controller, feed, "limelight");
  }

  public AlignTargetOdometry(
      CommandSwerveDrivetrain drivetrain,
      SwerveRequest.FieldCentric drive,
      CommandXboxController controller,
      boolean feed,
      String limelightName) {
    m_drivetrain = drivetrain;
    m_controller = controller;
    this.drive = drive;
    this.limelightName = limelightName;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_alignPID.reset();
    m_alignPID.enableContinuousInput(-180, 180);
    m_alignPID.setTolerance(ALIGNMENT_TOLERANCE_DEGREES);
    targetPose = AllianceUtil.getHubPose();
    m_rotationLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Determine target angle based on vision or odometry
    double targetAngleDegrees;
    boolean usingVision = false;
    
    if (hasValidVisionTarget()) {
      // Use Limelight for precise alignment
      targetAngleDegrees = getVisionTargetAngle();
      usingVision = true;
      
      // Get distance from Limelight for shooter
      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
      if (fiducials.length > 0) {
        distanceToTarget = fiducials[0].distToRobot;
      } else {
        distanceToTarget = Double.NaN;
      }
    } else {
      // Fall back to odometry-based alignment
      targetAngleDegrees = getOdometryTargetAngle();
      
      // Calculate distance from odometry for shooter
      Pose2d hubPose = AllianceUtil.isBlueAlliance() 
          ? Constants.FieldConstants.blueHubPose 
          : Constants.FieldConstants.redHubPose;
      Pose2d robotPose = m_drivetrain.getState().Pose;
      
      double dx = hubPose.getX() - robotPose.getX();
      double dy = hubPose.getY() - robotPose.getY();
      distanceToTarget = Math.sqrt(dx * dx + dy * dy);
    }

    // Get current heading
    double currentHeadingDegrees = m_drivetrain.getState().Pose.getRotation().getDegrees();
    
    // Calculate PID output
    double pidOutput = m_alignPID.calculate(currentHeadingDegrees, targetAngleDegrees);
    
    // Apply slew rate limiting for smooth motion
    double rotationRate = m_rotationLimiter.calculate(pidOutput * MaxAngularRate);
    
    // Clamp to max angular rate
    rotationRate = MathUtil.clamp(rotationRate, -MaxAngularRate, MaxAngularRate);

    // Apply control with driver input for translation
    m_drivetrain.setControl(drive
        .withVelocityX(-m_controller.getLeftY() * MaxSpeed)
        .withVelocityY(-m_controller.getLeftX() * MaxSpeed)
        .withRotationalRate(rotationRate));

    // Telemetry for tuning
    SmartDashboard.putBoolean("Align/UsingVision", usingVision);
    SmartDashboard.putNumber("Align/TargetAngle", targetAngleDegrees);
    SmartDashboard.putNumber("Align/CurrentAngle", currentHeadingDegrees);
    SmartDashboard.putNumber("Align/Error", targetAngleDegrees - currentHeadingDegrees);
    SmartDashboard.putNumber("Align/RotationRate", rotationRate);
    SmartDashboard.putBoolean("Align/AtSetpoint", m_alignPID.atSetpoint());
    SmartDashboard.putNumber("Align/DistanceToTarget", distanceToTarget);
  }

  /**
   * Check if Limelight has a valid target we care about
   */
  private boolean hasValidVisionTarget() {
    // Check if target is visible
    if (!LimelightHelpers.getTV(limelightName)) {
      return false;
    }

    // Check if target is large enough (close enough)
    double targetArea = LimelightHelpers.getTA(limelightName);
    if (targetArea < LIMELIGHT_MIN_AREA) {
      return false;
    }

    // Check if it's a valid alliance tag
    int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
    if (tagId < 0) {
      return false;
    }

    return m_drivetrain.isValidAllianceTag(tagId);
  }

  /**
   * Get target angle using Limelight
   * Returns the absolute field angle the robot should face
   */
  private double getVisionTargetAngle() {
    // Get horizontal offset (tx is positive when target is to the right)
    double tx = LimelightHelpers.getTX(limelightName);
    
    // Current robot heading
    double currentHeading = m_drivetrain.getState().Pose.getRotation().getDegrees();
    
    // Target angle is current heading plus offset
    double targetAngle = currentHeading + tx;
    
    // Normalize to [-180, 180]
    targetAngle = normalizeAngle(targetAngle);
    
    SmartDashboard.putNumber("Align/Vision/TX", tx);
    
    return targetAngle;
  }

  /**
   * Get target angle using odometry
   * Calculates the angle from robot to hub
   */
  private double getOdometryTargetAngle() {
    Pose2d hubPose;
    
    if (AllianceUtil.isBlueAlliance()) {
      hubPose = Constants.FieldConstants.blueHubPose;
    } else {
      hubPose = Constants.FieldConstants.redHubPose;
    }
    
    Pose2d robotPose = m_drivetrain.getState().Pose;
    
    // Calculate angle to target
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();
    
    return Units.radiansToDegrees(Math.atan2(dy, dx));
  }

  /**
   * Normalize angle to [-180, 180] range
   */
  private double normalizeAngle(double angleDegrees) {
    return Rotation2d.fromDegrees(angleDegrees).getDegrees();
  }

  @Override
  public void end(boolean interrupted) {
    // Don't stop the robot - allow driver to maintain control
    // The command will naturally stop controlling rotation when it ends
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Get the current distance to target in meters
   * Used by shooter subsystem to calculate launch velocity
   * @return distance in meters, or Double.NaN if no valid target
   */
  public double getDistanceToTarget() {
    return distanceToTarget;
  }
}