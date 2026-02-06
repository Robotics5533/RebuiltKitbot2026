package frc.robot.subsystems;

import java.util.Optional;

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

public class LimelightSubsystem extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    public LimelightSubsystem(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Use the current robot pose (odometry) to seed the MegaTag2 algorithm
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        if (
            poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null 
                || poseEstimate_MegaTag1.tagCount == 0 
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift,
        //    but ONLY if we have multiple tags to avoid ambiguity flips.
        
        Pose2d finalPose = poseEstimate_MegaTag2.pose;
        
        // Only use MegaTag1 rotation if we have high confidence (multiple tags)
        if (poseEstimate_MegaTag1.tagCount >= 2) {
             finalPose = new Pose2d(
                poseEstimate_MegaTag2.pose.getTranslation(),
                poseEstimate_MegaTag1.pose.getRotation()
            );
        }

        poseEstimate_MegaTag2.pose = finalPose;
        
        // Trust vision for X/Y (0.1m) but trust it less for rotation (10.0 deg/rad) to let Gyro dominate
        // MegaTag2 uses the Gyro for orientation, so we don't want to double-update rotation strongly 
        // unless we want to drift-correct the gyro using MT1.
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
    
    public String getName() {
        return name;
    }
    
    // Helper methods for other commands
    public boolean hasTarget() {
        return LimelightHelpers.getTV(name);
    }

    public double getTx() {
        return LimelightHelpers.getTX(name);
    }
    
    public double getTy() {
        return LimelightHelpers.getTY(name);
    }
}
