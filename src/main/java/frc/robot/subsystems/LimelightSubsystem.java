package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    
    private final String limelightName = Constants.LimelightConstants.LIMELIGHT_NAME;
    private final CommandSwerveDrivetrain drivetrain;

    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Update MegaTag2 with robot orientation
        // We use the swerve's odometry rotation (which is fused with gyro)
        Rotation2d rotation = drivetrain.getState().Pose.getRotation();
        double rotationalRateDegPerSec = Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        
        LimelightHelpers.SetRobotOrientation(limelightName, rotation.getDegrees(), rotationalRateDegPerSec, 0, 0, 0, 0);

        updateVisionMeasurement();
    }

    private void updateVisionMeasurement() {
        // Use MegaTag2 for high-accuracy pose estimation
        LimelightHelpers.PoseEstimate visionEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (visionEst.tagCount > 0) {
            double xyStdDev = 0.7;
            double degStdDev = 0.7;

            // If we have multiple tags, we can trust the estimate much more
            if (visionEst.tagCount >= 2) {
                xyStdDev = 0.1;
                degStdDev = 0.1;
            }
            // If we only have 1 tag, trust it if it's close
            else if (visionEst.avgTagDist < 4.0) {
                xyStdDev = 0.3;
                degStdDev = 0.3;
            }
            // Trust vision less if we are moving fast
            if (drivetrain.getState().Speeds.vxMetersPerSecond > 4.0 || drivetrain.getState().Speeds.omegaRadiansPerSecond > 2.0) {
                xyStdDev = 1.0;
                degStdDev = 1.0;
            }

            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, degStdDev));
            drivetrain.addVisionMeasurement(visionEst.pose, visionEst.timestampSeconds);
        }
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTx() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTy() {
        return LimelightHelpers.getTY(limelightName);
    }
    
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }
}
