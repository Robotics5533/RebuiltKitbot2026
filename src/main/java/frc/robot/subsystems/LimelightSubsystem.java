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
        // Seed MegaTag2 with current odometry rotation to avoid flipping
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // We use MegaTag2 for stability, as it uses the Gyro to resolve ambiguity
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return Optional.empty();
        }

        // Dynamic Standard Deviations based on Tag Count and Distance
        double xyStDev = 0.1; // Baseline: Trust vision for XY
        double degStDev = 999.0; // Baseline: Trust Gyro for Theta (ignore vision rotation)

        // If we have multiple tags, we can trust vision rotation slightly to correct long-term drift
        if (poseEstimate.tagCount >= 2) {
            degStDev = 5.0; 
        }
        
        // Penalize distant tags
        if (poseEstimate.avgTagDist > 3.0) {
            xyStDev *= (poseEstimate.avgTagDist / 3.0);
        }
        
        // Reject if ambiguity is too high (only relevant if using raw fiducials, but MT2 handles this internally)
        // However, if we have 1 tag and it's far away, MT2 is still reliant on good Gyro.
        
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(xyStDev, xyStDev, degStDev);

        posePublisher.set(poseEstimate.pose);

        return Optional.of(new Measurement(poseEstimate, standardDeviations));
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
