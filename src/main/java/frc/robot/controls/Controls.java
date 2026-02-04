package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.AutoAlignHub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Set;

public class Controls {
    private final CommandXboxController driver;
    private final CommandXboxController operator;

    public Controls() {
        driver = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
        operator = new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);
    }

    // Driver Inputs
    public double getDriveX() {
        return -MathUtil.applyDeadband(driver.getLeftY(), Constants.DriveConstants.DEADBAND);
    }

    public double getDriveY() {
        return -MathUtil.applyDeadband(driver.getLeftX(), Constants.DriveConstants.DEADBAND);
    }

    public double getDriveOmega() {
        return -MathUtil.applyDeadband(driver.getRightX(), Constants.DriveConstants.DEADBAND);
    }

    // Driver Bindings
    public void configureDriver(CommandSwerveDrivetrain drivetrain) {
        driver.rightBumper().whileTrue(
                new AutoAlignHub(drivetrain, driver));

        driver.leftBumper().onTrue(
                drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.a().whileTrue(
                drivetrain.applyRequest(() -> new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake()));

        driver.b().whileTrue(drivetrain.applyRequest(
                () -> new com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt().withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
    }

    public void configureDriverWithShooter(ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain) {
        driver.rightTrigger(Constants.OperatorConstants.TRIGGER_THRESHOLD).whileTrue(Commands.defer(() -> {
            var align = new AutoAlignHub(drivetrain, driver);
            return Commands.parallel(
                    align,
                    Commands.sequence(
                            shooter.spinUpCommand().until(() -> align.isAligned() && shooter.isLauncherAtSpeed()),
                            shooter.launchCommand()),
                    Commands.run(() -> shooter.setTargetDistance(align.getDistanceToTarget())));
        }, Set.of(shooter)).withName("Driver Align and Shoot"));
    }

    // Operator Bindings
    public void configureOperator(ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain) {
        operator.leftBumper().whileTrue(shooter.intakeCommand());
        operator.rightBumper().whileTrue(shooter.launchCommand());

        operator.rightTrigger(Constants.OperatorConstants.TRIGGER_THRESHOLD).whileTrue(Commands.defer(() -> {
            var align = new AutoAlignHub(drivetrain, driver);
            return Commands.parallel(
                    align,
                    Commands.sequence(
                            shooter.spinUpCommand().until(() -> align.isAligned() && shooter.isLauncherAtSpeed()),
                            shooter.launchCommand()),
                    Commands.run(() -> shooter.setTargetDistance(align.getDistanceToTarget())));
        }, Set.of(shooter)).withName("Align and Shoot"));

        operator.y().whileTrue(new AutoAlignHub(drivetrain, driver));

        operator.a().whileTrue(Commands.defer(() -> {
            var align = new AutoAlignHub(drivetrain, driver);
            return Commands.parallel(
                    align,
                    shooter.spinUpAndShootCommand(),
                    Commands.run(() -> shooter.setTargetDistance(align.getDistanceToTarget())));
        }, Set.of(shooter)).withName("Auto Shoot"));

        // SysId Controls: Start + A for launcher quasistatic test, Start + B for feeder quasistatic test
        // Quasistatic tests slowly ramp voltage from lowest to highest, providing smoother characterization
        operator.start().and(operator.a()).whileTrue(
                shooter.launcherSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward)
                        .withName("Launcher SysId Quasistatic Forward"));

        operator.start().and(operator.b()).whileTrue(
                shooter.feederSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward)
                        .withName("Feeder SysId Quasistatic Forward"));
    }

    public CommandXboxController getDriver() {
        return driver;
    }

    public CommandXboxController getOperator() {
        return operator;
    }
}
