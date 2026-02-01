// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final class RobotConstants {
        public static final double robotOverallLength = Units.inchesToMeters(36.0);
        public static final double robotOverallWidth = Units.inchesToMeters(34.0);

         public static final InterpolatingDoubleTreeMap FuelVelocity = new InterpolatingDoubleTreeMap();
    static {
      // Example data points - replace with your actual tuned values
      FuelVelocity.put(1.0, 2500.0);  // 1.0m -> 2500 RPM
      FuelVelocity.put(1.5, 2900.0);  // 1.5m -> 2900 RPM
      FuelVelocity.put(2.0, 3200.0);  // 2.0m -> 3200 RPM
      FuelVelocity.put(2.5, 3500.0);  // 2.5m -> 3500 RPM
      FuelVelocity.put(3.0, 3800.0);  // 3.0m -> 3800 RPM
      FuelVelocity.put(3.5, 4100.0);  // 3.5m -> 4100 RPM
      FuelVelocity.put(4.0, 4300.0);  // 4.0m -> 4300 RPM
      FuelVelocity.put(4.5, 4500.0);  // 4.5m -> 4500 RPM
      FuelVelocity.put(5.0, 4700.0);  // 5.0m -> 4700 RPM
    }
  
    }

    public static final class LimelightConstants {
        public static final Set<Integer> RED_HUB_TAGS = Set.of(11, 2, 9, 10, 8, 5);

        public static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 27, 26, 25, 21, 24);

        public static final String LIMELIGHT_NAME = "limelight";
    }

    

    public static final class FieldConstants {

        static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

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

    }

}