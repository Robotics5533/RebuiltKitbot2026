package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {

  private final String name;
  private final NetworkTable telemetryTable;
  private final StructPublisher<Pose2d> posePublisher;

  private final CommandSwerveDrivetrain drivetrain;

  public LimelightSubsystem(String name, CommandSwerveDrivetrain drivetrain) {
    this.name = name;
    this.drivetrain = drivetrain;

    telemetryTable = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard/" + name);

    posePublisher = telemetryTable
        .getStructTopic("Estimated Robot Pose", Pose2d.struct)
        .publish();
  }   

  public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {

    
    double yawDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();

    LimelightHelpers.SetRobotOrientation(
        name,
        yawDeg,
        0, 0, 0, 0, 0);

    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    
    if (poseEstimate == null || poseEstimate.tagCount < 1) {
      return Optional.empty();
    }

    
    double speed = Math.hypot(
        drivetrain.getState().Speeds.vxMetersPerSecond,
        drivetrain.getState().Speeds.vyMetersPerSecond);

    
    double xyStDev = 0.25 * poseEstimate.avgTagDist;
    double degStDev = 8.0;

    
    if (speed > 0.5) {
      xyStDev *= 1.5;
      degStDev *= 1.5;
    }

    
    if (poseEstimate.tagCount == 1) {
      xyStDev *= 2.0;
      degStDev *= 2.0;
    }

    
    
    
    Matrix<N3, N1> standardDeviations = VecBuilder.fill(xyStDev, xyStDev, Double.MAX_VALUE);

    
    
    Pose2d correctedPose = new Pose2d(
        poseEstimate.pose.getTranslation(),
        poseEstimate.pose.getRotation().plus(Rotation2d.fromDegrees(180))
    );

    posePublisher.set(correctedPose);

    
    PoseEstimate correctedEstimate = new PoseEstimate();
    correctedEstimate.pose = correctedPose;
    correctedEstimate.timestampSeconds = poseEstimate.timestampSeconds;
    correctedEstimate.tagCount = poseEstimate.tagCount;
    correctedEstimate.tagSpan = poseEstimate.tagSpan;
    correctedEstimate.avgTagDist = poseEstimate.avgTagDist;
    correctedEstimate.avgTagArea = poseEstimate.avgTagArea;
    correctedEstimate.rawFiducials = poseEstimate.rawFiducials;

    return Optional.of(new Measurement(correctedEstimate, standardDeviations));
  }

  public static class Measurement {
    public final PoseEstimate poseEstimate;
    public final Matrix<N3, N1> standardDeviations;

    public Measurement(PoseEstimate poseEstimate,
        Matrix<N3, N1> standardDeviations) {
      this.poseEstimate = poseEstimate;
      this.standardDeviations = standardDeviations;
    }
  }

  
  public boolean hasTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getTx() {
    return LimelightHelpers.getTX(name);
  }

  public double getTy() {
    return LimelightHelpers.getTY(name);
  }

  public double getTa() {
    return LimelightHelpers.getTA(name);
  }

}