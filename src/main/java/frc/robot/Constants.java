package frc.robot;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class RobotConstants {
        public static final double robotOverallLength = Units.inchesToMeters(36.0);
        public static final double robotOverallWidth = Units.inchesToMeters(34.0);

    }

    public static final class ShooterConstants {
        public static final int FEEDER_MOTOR_ID = 14;
        public static final int LAUNCHER_MOTOR_ID = 15;

        public static final double INTAKING_FEEDER_RPM = 3000;
        public static final double INTAKING_LAUNCHER_RPM = -2500;
        public static final double IDLE_FEEDER_RPM = 1500;

        public static final double LAUNCHER_kP = 0.0002;
        public static final double LAUNCHER_kI = 0.0;
        public static final double LAUNCHER_kD = 0.0;
        public static final double LAUNCHER_kFF = 0.000175;

        public static final double FEEDER_kP = 0.0001;
        public static final double FEEDER_kI = 0.0;
        public static final double FEEDER_kD = 0.0;
        public static final double FEEDER_kFF = 0.000175;

        public static final double VELOCITY_TOLERANCE_RPM = 100;
        public static final double MANUAL_LAUNCHER_RPM = 4000;

        public static final int FEEDER_CURRENT_LIMIT = 30;
        public static final int LAUNCHER_CURRENT_LIMIT = 40;

        public static final InterpolatingDoubleTreeMap FuelVelocity = new InterpolatingDoubleTreeMap();
        static {
            FuelVelocity.put(1.0, 2500.0);
            FuelVelocity.put(1.5, 2900.0);
            FuelVelocity.put(2.0, 3200.0);
            FuelVelocity.put(2.5, 3500.0);
            FuelVelocity.put(3.0, 3800.0);
            FuelVelocity.put(3.5, 4100.0);
            FuelVelocity.put(4.0, 4300.0);
            FuelVelocity.put(4.5, 4500.0);
            FuelVelocity.put(5.0, 4700.0);
        }
    }

    public static final class DriveConstants {
        public static final double DEADBAND = 0.05;
        public static final double ALIGN_PID_P = 0.025;
        public static final double ALIGN_PID_I = 0.0;
        public static final double ALIGN_PID_D = 0.008;
        public static final double ALIGN_TOLERANCE_DEG = 2.0;
        public static final double ALIGN_ROTATION_LIMIT = 4.0;
    }

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double TRIGGER_THRESHOLD = 0.5;
    }

    public static final class LimelightConstants {
        public static final Set<Integer> RED_HUB_TAGS = Set.of(11, 2, 9, 10, 8, 5);
        public static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 27, 26, 25, 21, 24);
        public static final String LIMELIGHT_NAME = "limelight";
    }

    public static final class FieldConstants {
        static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltAndymark);

        public static double fieldLength = aprilTagFieldLayout.getFieldLength();
        public static double fieldWidth = aprilTagFieldLayout.getFieldWidth();

        static double blueStartingLineX = Units.inchesToMeters(156);
        static double blueOutpostSideTrenchStartY = Units.inchesToMeters(24.85);
        static double blueDepotSideTrenchStartY = Units.inchesToMeters(fieldWidth + Units.inchesToMeters(24.85));

        static Pose2d blueOutpostSideTrenchStart = new Pose2d(
                blueStartingLineX - RobotConstants.robotOverallLength / 2,
                blueOutpostSideTrenchStartY,
                new Rotation2d());

        static Pose2d blueDepotSideTrenchStart = new Pose2d(
                blueStartingLineX - RobotConstants.robotOverallLength / 2,
                blueDepotSideTrenchStartY,
                new Rotation2d());

        static Pose2d blueCenterStart = new Pose2d(
                blueStartingLineX - RobotConstants.robotOverallLength / 2,
                Units.inchesToMeters(fieldWidth / 2),
                new Rotation2d());

        public static Pose2d blueHubPose = new Pose2d(
                Units.inchesToMeters(181.56), FieldConstants.fieldWidth / 2, new Rotation2d());

        static double redStartingLineX = Units.inchesToMeters(fieldLength - Units.inchesToMeters(143.5));

        public static Pose2d redHubPose = new Pose2d(
                FieldConstants.fieldLength - Units.inchesToMeters(181.56), FieldConstants.fieldWidth / 2,
                new Rotation2d(Math.PI));

        public static Pose2d blueTowerRightPose = new Pose2d(
                blueHubPose.getX() - Units.inchesToMeters(151),
                blueHubPose.getY() - Units.inchesToMeters(45),
                new Rotation2d());

        public static Pose2d redTowerRightPose = new Pose2d(
                redHubPose.getX() + Units.inchesToMeters(151),
                redHubPose.getY() + Units.inchesToMeters(45),
                new Rotation2d(Math.PI));
    }
}
