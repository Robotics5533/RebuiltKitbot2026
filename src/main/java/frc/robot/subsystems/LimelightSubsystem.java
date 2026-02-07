package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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

  public LimelightSubsystem(String name) {
    this.name = name;
    this.telemetryTable =
        NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
    this.posePublisher =
        telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct)
            .publish();
  }

  public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
    LimelightHelpers.SetRobotOrientation(
        name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate poseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (poseEstimate == null || poseEstimate.tagCount == 0) {
      return Optional.empty();
    }

    double xyStDev = 0.1;
    double degStDev = 999.0;

    if (poseEstimate.tagCount >= 2) {
      degStDev = 5.0;
    }

    if (poseEstimate.avgTagDist > 3.0) {
      xyStDev *= (poseEstimate.avgTagDist / 3.0);
    }

    final Matrix<N3, N1> standardDeviations =
        VecBuilder.fill(xyStDev, xyStDev, degStDev);

    posePublisher.set(poseEstimate.pose);

    return Optional.of(new Measurement(poseEstimate, standardDeviations));
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

  public String getName() { return name; }

  public boolean hasTarget() { return LimelightHelpers.getTV(name); }

  public double getTx() { return LimelightHelpers.getTX(name); }

  public double getTy() { return LimelightHelpers.getTY(name); }
}
