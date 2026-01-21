package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShootLoad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {



  private final double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Swerve requests */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake =
      new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point =
      new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentric forwardStraight =
      new SwerveRequest.FieldCentric().withDriveRequestType(
          DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain =
      TunerConstants.createDrivetrain();
  public final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand(
      "shoot_load",
      ShootLoad.shootForTime(fuelSubsystem, 4.0)
  );
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);
    

    configureBindings();

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.run(() -> {
      double vx = -MathUtil.applyDeadband(joystick.getLeftY(), 0.05) * MaxSpeed;
      double vy = -MathUtil.applyDeadband(joystick.getLeftX(), 0.05) * MaxSpeed;
      double manualOmega =
          -MathUtil.applyDeadband(joystick.getRightX(), 0.05) * MaxAngularRate;

      boolean autoAim = joystick.rightBumper().getAsBoolean();

      drivetrain.driveWithAutoAim(drive, vx, vy, manualOmega, autoAim);
    }));

    // Idle while robot is disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Button bindings
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    joystick.b().whileTrue(drivetrain.applyRequest(
        ()
            -> point.withModuleDirection(
                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // joystick.povUp().whileTrue(drivetrain.applyRequest(
    //     () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // joystick.povDown().whileTrue(drivetrain.applyRequest(
    //     () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    // joystick.povLeft().whileTrue(drivetrain.applyRequest(
    //     () -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));
    // joystick.povRight().whileTrue(drivetrain.applyRequest(
    //     () -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));

    // SysId routines
    // joystick.back()
    //     .and(joystick.y())
    //     .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back()
    //     .and(joystick.x())
    //     .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start()
    //     .and(joystick.y())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start()
    //     .and(joystick.x())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset field-centric heading
    joystick.leftBumper().onTrue(
        drivetrain.runOnce(drivetrain::seedFieldCentric));

    operator.leftBumper().whileTrue(fuelSubsystem.intakeCommand());
    operator.rightBumper().whileTrue(fuelSubsystem.launchCommand());


    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() { return autoChooser.getSelected(); }
}
