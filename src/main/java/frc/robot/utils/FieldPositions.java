package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import frc.robot.Constants;

public final class FieldPositions {

    private static final String ROOT2D = "/FieldPoses2d/";
    private static final String ROOT3D = "/FieldPoses3d/";

    private static final StructSubscriber<Pose2d> blueHub2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueHub", Pose2d.struct)
            .subscribe(Constants.FieldConstants.blueHubPose);

    private static final StructSubscriber<Pose2d> redHub2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedHub", Pose2d.struct)
            .subscribe(Constants.FieldConstants.redHubPose);

    private static final StructSubscriber<Pose2d> blueTowerRight2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueTowerRight", Pose2d.struct)
            .subscribe(Constants.FieldConstants.blueTowerRightPose);

    private static final StructSubscriber<Pose2d> redTowerRight2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedTowerRight", Pose2d.struct)
            .subscribe(Constants.FieldConstants.redTowerRightPose);

    private static final StructSubscriber<Pose2d> blueBumpLeft2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueBumpLeft", Pose2d.struct)
            .subscribe(Constants.FieldConstants.blueBumpLeftPose);

    private static final StructSubscriber<Pose2d> blueBumpRight2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueBumpRight", Pose2d.struct)
            .subscribe(Constants.FieldConstants.blueBumpRightPose);

    private static final StructSubscriber<Pose2d> redBumpLeft2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedBumpLeft", Pose2d.struct)
            .subscribe(Constants.FieldConstants.redBumpLeftPose);

    private static final StructSubscriber<Pose2d> redBumpRight2dSub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedBumpRight", Pose2d.struct)
            .subscribe(Constants.FieldConstants.redBumpRightPose);

    private static final StructPublisher<Pose2d> blueHub2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueHub", Pose2d.struct).publish();

    private static final StructPublisher<Pose2d> redHub2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedHub", Pose2d.struct).publish();

    private static final StructPublisher<Pose2d> blueTowerRight2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueTowerRight", Pose2d.struct).publish();

    private static final StructPublisher<Pose2d> redTowerRight2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedTowerRight", Pose2d.struct).publish();

    private static final StructPublisher<Pose2d> blueBumpLeft2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueBumpLeft", Pose2d.struct).publish();

    private static final StructPublisher<Pose2d> blueBumpRight2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "BlueBumpRight", Pose2d.struct).publish();

    private static final StructPublisher<Pose2d> redBumpLeft2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedBumpLeft", Pose2d.struct).publish();

    private static final StructPublisher<Pose2d> redBumpRight2dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT2D + "RedBumpRight", Pose2d.struct).publish();

    private static final StructPublisher<Pose3d> blueHub3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "BlueHub", Pose3d.struct).publish();

    private static final StructPublisher<Pose3d> redHub3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "RedHub", Pose3d.struct).publish();

    private static final StructPublisher<Pose3d> blueTowerRight3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "BlueTowerRight", Pose3d.struct).publish();

    private static final StructPublisher<Pose3d> redTowerRight3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "RedTowerRight", Pose3d.struct).publish();

    private static final StructPublisher<Pose3d> blueBumpLeft3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "BlueBumpLeft", Pose3d.struct).publish();

    private static final StructPublisher<Pose3d> blueBumpRight3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "BlueBumpRight", Pose3d.struct).publish();

    private static final StructPublisher<Pose3d> redBumpLeft3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "RedBumpLeft", Pose3d.struct).publish();

    private static final StructPublisher<Pose3d> redBumpRight3dPub = NetworkTableInstance.getDefault()
            .getStructTopic(ROOT3D + "RedBumpRight", Pose3d.struct).publish();

    static {
        blueHub2dPub.set(Constants.FieldConstants.blueHubPose);
        redHub2dPub.set(Constants.FieldConstants.redHubPose);
        blueTowerRight2dPub.set(Constants.FieldConstants.blueTowerRightPose);
        redTowerRight2dPub.set(Constants.FieldConstants.redTowerRightPose);
        blueBumpLeft2dPub.set(Constants.FieldConstants.blueBumpLeftPose);
        blueBumpRight2dPub.set(Constants.FieldConstants.blueBumpRightPose);
        redBumpLeft2dPub.set(Constants.FieldConstants.redBumpLeftPose);
        redBumpRight2dPub.set(Constants.FieldConstants.redBumpRightPose);

        blueHub3dPub.set(new Pose3d(Constants.FieldConstants.blueHubPose));
        redHub3dPub.set(new Pose3d(Constants.FieldConstants.redHubPose));
        blueTowerRight3dPub.set(new Pose3d(Constants.FieldConstants.blueTowerRightPose));
        redTowerRight3dPub.set(new Pose3d(Constants.FieldConstants.redTowerRightPose));
        blueBumpLeft3dPub.set(new Pose3d(Constants.FieldConstants.blueBumpLeftPose));
        blueBumpRight3dPub.set(new Pose3d(Constants.FieldConstants.blueBumpRightPose));
        redBumpLeft3dPub.set(new Pose3d(Constants.FieldConstants.redBumpLeftPose));
        redBumpRight3dPub.set(new Pose3d(Constants.FieldConstants.redBumpRightPose));
    }

    public static Pose2d getBlueHubPose() {
        return blueHub2dSub.get();
    }

    public static Pose2d getRedHubPose() {
        return redHub2dSub.get();
    }

    public static Pose2d getBlueTowerRightPose() {
        return blueTowerRight2dSub.get();
    }

    public static Pose2d getRedTowerRightPose() {
        return redTowerRight2dSub.get();
    }

    public static Pose2d getBlueBumpLeftPose() {
        return blueBumpLeft2dSub.get();
    }

    public static Pose2d getBlueBumpRightPose() {
        return blueBumpRight2dSub.get();
    }

    public static Pose2d getRedBumpLeftPose() {
        return redBumpLeft2dSub.get();
    }

    public static Pose2d getRedBumpRightPose() {
        return redBumpRight2dSub.get();
    }
}
