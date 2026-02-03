package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoAlignHub;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.controls.Controls;

public class RobotContainer {

    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.DriveConstants.DEADBAND)
            .withRotationalDeadband(MaxAngularRate * Constants.DriveConstants.DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Controls controls = new Controls();
    private final Field2d fieldViz = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand(
                "shoot_load",
                Commands.sequence(
                    Commands.waitUntil(() -> AutoAlignHub.isAligned(drivetrain)),
                    shooter.launchCommand()
                ).withTimeout(2.5));
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Field", fieldViz);

        configureBindings();
        FollowPathCommand.warmupCommand();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.run(() -> {
            drivetrain.setControl(
                    drive.withVelocityX(controls.getDriveX() * MaxSpeed)
                            .withVelocityY(controls.getDriveY() * MaxSpeed)
                            .withRotationalRate(controls.getDriveOmega() * MaxAngularRate));
            fieldViz.setRobotPose(drivetrain.getState().Pose);
            fieldViz.getObject("BlueHub").setPose(frc.robot.utils.FieldPositions.getBlueHubPose());
            fieldViz.getObject("RedHub").setPose(frc.robot.utils.FieldPositions.getRedHubPose());
            fieldViz.getObject("BlueTowerRight").setPose(frc.robot.utils.FieldPositions.getBlueTowerRightPose());
            fieldViz.getObject("RedTowerRight").setPose(frc.robot.utils.FieldPositions.getRedTowerRightPose());
        }));

        controls.configureDriver(drivetrain);
        controls.configureDriverWithShooter(shooter, drivetrain);
        controls.configureOperator(shooter, drivetrain);

        controls.getDriver().y().onTrue(pathfindToRightTower());

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command pathfindToRightTower() {
        var target = frc.robot.utils.AllianceUtil.isRedAlliance()
                ? frc.robot.utils.FieldPositions.getRedTowerRightPose()
                : frc.robot.utils.FieldPositions.getBlueTowerRightPose();
        PathConstraints constraints = new PathConstraints(
                MaxSpeed * 0.8,
                MaxSpeed * 1.2,
                MaxAngularRate * 0.8,
                MaxAngularRate * 1.2);
        return AutoBuilder.pathfindToPose(target, constraints);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
