package frc.robot.controls;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.AutoAlignHub;
import frc.robot.commands.AutoTuneRotation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Set;

public class Controls {
    private final CommandXboxController driver;
    private final CommandXboxController operator;

    public Controls() {
        driver = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
        operator = new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);
    }

    
    public double getDriveX() {
        return -MathUtil.applyDeadband(driver.getLeftY(), Constants.DriveConstants.DEADBAND);
    }

    public double getDriveY() {
        return -MathUtil.applyDeadband(driver.getLeftX(), Constants.DriveConstants.DEADBAND);
    }

    public double getDriveOmega() {
        return -MathUtil.applyDeadband(driver.getRightX(), Constants.DriveConstants.DEADBAND);
    }

    
    public void configureDriver(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
        driver.rightBumper().whileTrue(
                new AutoAlignHub(drivetrain, limelight, driver));

        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.a().whileTrue(
                drivetrain.applyRequest(() -> new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake()));

        driver.b().whileTrue(drivetrain.applyRequest(
                () -> new com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt().withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        
        driver.start().whileTrue(new AutoTuneRotation(drivetrain));
    }

    public void configureDriverWithShooter(ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
        driver.rightTrigger(Constants.OperatorConstants.TRIGGER_THRESHOLD).whileTrue(Commands.defer(() -> {
            var align = new AutoAlignHub(drivetrain, limelight, driver);
            return Commands.parallel(
                    align,
                    Commands.sequence(
                            shooter.spinUpCommand().until(() -> align.isAligned() && shooter.isLauncherAtSpeed()),
                            shooter.launchCommand()),
                    Commands.run(() -> shooter.setTargetDistance(align.getDistanceToTarget())));
        }, Set.of(shooter)).withName("Driver Align and Shoot"));
    }

    
    public void configureOperator(ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
        operator.leftBumper().whileTrue(shooter.intakeCommand());
        operator.rightBumper().whileTrue(shooter.launchCommand());

        operator.rightTrigger(Constants.OperatorConstants.TRIGGER_THRESHOLD).whileTrue(Commands.defer(() -> {
            var align = new AutoAlignHub(drivetrain, limelight, driver);
            return Commands.parallel(
                    align,
                    Commands.sequence(
                            shooter.spinUpCommand().until(() -> align.isAligned() && shooter.isLauncherAtSpeed()),
                            shooter.launchCommand()),
                    Commands.run(() -> shooter.setTargetDistance(align.getDistanceToTarget())));
        }, Set.of(shooter)).withName("Align and Shoot"));

        operator.y().whileTrue(new AutoAlignHub(drivetrain, limelight, driver));

        operator.a().whileTrue(Commands.defer(() -> {
            var align = new AutoAlignHub(drivetrain, limelight, driver);
            return Commands.parallel(
                    align,
                    shooter.spinUpAndShootCommand(),
                    Commands.run(() -> shooter.setTargetDistance(align.getDistanceToTarget())));
        }, Set.of(shooter)).withName("Auto Shoot"));
    }

    public CommandXboxController getDriver() {
        return driver;
    }

    public CommandXboxController getOperator() {
        return operator;
    }
}