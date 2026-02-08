package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoAlignHub;
import frc.robot.controls.Controls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.FieldPositions;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.MathUtil;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.DriveConstants.DEADBAND)
            .withRotationalDeadband(MaxAngularRate *
                    Constants.DriveConstants.DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Controls controls = new Controls();
    private final Field2d fieldViz = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelight = new LimelightSubsystem(Constants.LimelightConstants.LIMELIGHT_NAME,
            drivetrain);
    public final ShooterSubsystem shooter = new ShooterSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        
        
        LimelightHelpers.setCameraPose_RobotSpace(
                Constants.LimelightConstants.LIMELIGHT_NAME,
                -0.1905,  
                -0.3175,  
                0.3112,   
                180.0,    
                0.0,      
                180.0     
        );

        NamedCommands.registerCommand(
                "shoot_load",
                Commands
                        .sequence(
                                Commands.waitUntil(() -> AutoAlignHub.isAligned(drivetrain)),
                                shooter.launchCommand())
                        .withTimeout(2.5));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Field", fieldViz);

        configureBindings();
        FollowPathCommand.warmupCommand();
    }

    private void configureBindings() {
        limelight.setDefaultCommand(updateVisionCommand());

        drivetrain.setDefaultCommand(drivetrain.run(() -> {
            
            drivetrain.setControl(
                    drive.withVelocityX(controls.getDriveX() * MaxSpeed)
                            .withVelocityY(controls.getDriveY() * MaxSpeed)
                            .withRotationalRate(controls.getDriveOmega() * MaxAngularRate));

            
            fieldViz.setRobotPose(drivetrain.getState().Pose);

            fieldViz.getObject("BlueHub").setPose(FieldPositions.getBlueHubPose());
            fieldViz.getObject("RedHub").setPose(FieldPositions.getRedHubPose());

            fieldViz.getObject("BlueTowerRight")
                    .setPose(FieldPositions.getBlueTowerRightPose());
            fieldViz.getObject("RedTowerRight")
                    .setPose(FieldPositions.getRedTowerRightPose());

            fieldViz.getObject("BlueBumpLeft")
                    .setPose(FieldPositions.getBlueBumpLeftPose());
            fieldViz.getObject("BlueBumpRight")
                    .setPose(FieldPositions.getBlueBumpRightPose());
            fieldViz.getObject("RedBumpLeft")
                    .setPose(FieldPositions.getRedBumpLeftPose());
            fieldViz.getObject("RedBumpRight")
                    .setPose(FieldPositions.getRedBumpRightPose());

            updateDynamicObstacles();
        }));

        controls.configureDriver(drivetrain, limelight);
        controls.configureDriverWithShooter(shooter, drivetrain, limelight);
        controls.configureOperator(shooter, drivetrain, limelight);

        controls.getDriver().y().onTrue(pathfindToRightTower());

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void updateDynamicObstacles() {
        List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();

        obstacles.add(MathUtil.createBoundingBox(
                Constants.FieldConstants.blueBumpLeftPose.getTranslation(),
                Constants.FieldConstants.bumpWidth,
                Constants.FieldConstants.bumpDepth));

        obstacles.add(MathUtil.createBoundingBox(
                Constants.FieldConstants.blueBumpRightPose.getTranslation(),
                Constants.FieldConstants.bumpWidth,
                Constants.FieldConstants.bumpDepth));

        obstacles.add(MathUtil.createBoundingBox(
                Constants.FieldConstants.redBumpLeftPose.getTranslation(),
                Constants.FieldConstants.bumpWidth,
                Constants.FieldConstants.bumpDepth));

        obstacles.add(MathUtil.createBoundingBox(
                Constants.FieldConstants.redBumpRightPose.getTranslation(),
                Constants.FieldConstants.bumpWidth,
                Constants.FieldConstants.bumpDepth));

        Pathfinding.setDynamicObstacles(
                obstacles, drivetrain.getState().Pose.getTranslation());
    }

    public Command pathfindToRightTower() {
        var target = FieldPositions.getBlueTowerRightPose();

        PathConstraints constraints = new PathConstraints(MaxSpeed * 0.8, MaxSpeed * 1.2,
                MaxAngularRate * 0.8, MaxAngularRate * 1.2);

        return AutoBuilder.pathfindToPose(target, constraints, 0.0);
    }

    public Command updateVisionCommand() {
        return Commands.run(() -> {
            var measurement = limelight.getMeasurement(drivetrain.getState().Pose);

            if (measurement.isPresent()) {
                var m = measurement.get();

                drivetrain.setVisionMeasurementStdDevs(m.standardDeviations);
                drivetrain.addVisionMeasurement(
                        m.poseEstimate.pose,
                        m.poseEstimate.timestampSeconds);
            }
        }, limelight);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}