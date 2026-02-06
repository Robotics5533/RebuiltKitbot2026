package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax feederMotor;
    private final SparkMax launcherMotor;
    
    // Dashboard target for testing
    private double dashboardTargetRPM = Constants.ShooterConstants.MANUAL_LAUNCHER_RPM;

    public ShooterSubsystem() {
        feederMotor = createFeederMotor();
        launcherMotor = createLauncherMotor();
        
        SmartDashboard.putNumber("Shooter/Manual Launcher RPM", dashboardTargetRPM);
    }

    private SparkMax createFeederMotor() {
        var motor = new SparkMax(Constants.ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        
        config.smartCurrentLimit(Constants.ShooterConstants.FEEDER_CURRENT_LIMIT);
        
        // Configure PID
        config.closedLoop
            .p(Constants.ShooterConstants.FEEDER_kP)
            .i(Constants.ShooterConstants.FEEDER_kI)
            .d(Constants.ShooterConstants.FEEDER_kD)
            .outputRange(-1, 1);
            
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        return motor;
    }

    private SparkMax createLauncherMotor() {
        var motor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        
        config.inverted(true);
        config.smartCurrentLimit(Constants.ShooterConstants.LAUNCHER_CURRENT_LIMIT);
        
        // Configure PID
        config.closedLoop
            .p(Constants.ShooterConstants.LAUNCHER_kP)
            .i(Constants.ShooterConstants.LAUNCHER_kI)
            .d(Constants.ShooterConstants.LAUNCHER_kD)
            .outputRange(-1, 1);
            
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        return motor;
    }

    public void setLauncherRPM(double rpm) {
        // Use slot 0 for velocity control with ArbFF
        launcherMotor.getClosedLoopController().setReference(
            rpm, 
            ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0, 
            Constants.ShooterConstants.LAUNCHER_kV * rpm
        );
    }

    public void setFeederRPM(double rpm) {
        feederMotor.getClosedLoopController().setReference(
            rpm, 
            ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0, 
            Constants.ShooterConstants.FEEDER_kV * rpm
        );
    }
    
    public void setFeederVoltage(double volts) {
        feederMotor.setVoltage(volts);
    }

    public void stop() {
        launcherMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public double getLauncherVelocity() {
        return launcherMotor.getEncoder().getVelocity();
    }

    public boolean isLauncherAtSpeed(double targetRPM) {
        return Math.abs(getLauncherVelocity() - targetRPM) < Constants.ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }
    
    public boolean isLauncherAtSpeed() {
        // Only works if we are tracking the target somewhere, but for commands we pass the target
        // We can infer target from the last setpoint if we stored it, but reading back from SparkMax is async/delayed
        // Better to pass target or store local variable.
        // For this method, we'll use the dashboard target or assume the command handles it.
        // But to match the reference logic `isVelocityWithinTolerance`, we need the target.
        // I will add a local variable to track setpoint.
        return Math.abs(getLauncherVelocity() - currentLauncherSetpoint) < Constants.ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }
    
    private double currentLauncherSetpoint = 0;

    public Command spinUpCommand(DoubleSupplier rpmSupplier) {
        return run(() -> {
            double target = rpmSupplier.getAsDouble();
            currentLauncherSetpoint = target;
            setLauncherRPM(target);
            setFeederRPM(Constants.ShooterConstants.IDLE_FEEDER_RPM);
        })
        .until(() -> isLauncherAtSpeed(rpmSupplier.getAsDouble()))
        .withName("SpinUp");
    }
    
    public Command spinUpCommand() {
        return spinUpCommand(() -> getTargetRPM());
    }

    public Command launchCommand() {
        return run(() -> {
            // Keep spinning launcher
            setLauncherRPM(currentLauncherSetpoint);
            // Feed
            setFeederRPM(Constants.ShooterConstants.INTAKING_FEEDER_RPM); // Using intake speed for feeding
        }).withName("Launch");
    }
    
    public Command intakeCommand() {
        return run(() -> {
            setLauncherRPM(Constants.ShooterConstants.INTAKING_LAUNCHER_RPM);
            setFeederRPM(Constants.ShooterConstants.INTAKING_FEEDER_RPM);
        }).withName("Intake");
    }
    
    public Command spinUpAndShootCommand() {
        return spinUpCommand().andThen(launchCommand()).withName("SpinUpAndShoot");
    }
    
    public Command stopCommand() {
        return runOnce(this::stop).withName("Stop");
    }
    
    public double getTargetRPM() {
         double manual = SmartDashboard.getNumber("Shooter/Manual Launcher RPM", Constants.ShooterConstants.MANUAL_LAUNCHER_RPM);
         dashboardTargetRPM = manual;
         return manual;
    }
    
    // For auto setpoints
    public void setTargetDistance(Double distance) {
        // Calculate RPM based on distance and update dashboard/local variable if needed
        // For now, adhering to the simple logic requested.
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Launcher RPM", getLauncherVelocity());
        SmartDashboard.putNumber("Shooter/Feeder RPM", feederMotor.getEncoder().getVelocity());
    }
}
