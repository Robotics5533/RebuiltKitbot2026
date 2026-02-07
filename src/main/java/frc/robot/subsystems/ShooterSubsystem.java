package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    
    // Dashboard Keys
    private static final String KEY_INTAKE_FEEDER = "Shooter/Intaking Feeder Volts";
    private static final String KEY_INTAKE_LAUNCHER = "Shooter/Intaking Launcher Volts";
    private static final String KEY_LAUNCH_FEEDER = "Shooter/Launching Feeder Volts";
    private static final String KEY_LAUNCH_LAUNCHER = "Shooter/Launching Launcher Volts"; // Manual Target
    private static final String KEY_SPINUP_FEEDER = "Shooter/Spin-up Feeder Volts";
    
    private final SparkMax feederMotor;
    private final SparkMax launcherMotor;
    
    private double currentDistance = 0.0;
    private boolean useDistanceTuning = false; // Flag to enable distance-based tuning

    public ShooterSubsystem() {
        feederMotor = createFeederMotor();
        launcherMotor = createLauncherMotor();
        
        // Initialize Dashboard with Constants
        SmartDashboard.putNumber(KEY_INTAKE_FEEDER, Constants.ShooterConstants.INTAKING_FEEDER_VOLTS);
        SmartDashboard.putNumber(KEY_INTAKE_LAUNCHER, Constants.ShooterConstants.INTAKING_LAUNCHER_VOLTS);
        SmartDashboard.putNumber(KEY_LAUNCH_FEEDER, Constants.ShooterConstants.INTAKING_FEEDER_VOLTS); // Usually same as intaking for feeding? Or 8V
        SmartDashboard.putNumber(KEY_LAUNCH_LAUNCHER, Constants.ShooterConstants.MANUAL_LAUNCHER_VOLTS);
        SmartDashboard.putNumber(KEY_SPINUP_FEEDER, Constants.ShooterConstants.IDLE_FEEDER_VOLTS);
        SmartDashboard.putBoolean("Shooter/Use Distance Tuning", useDistanceTuning);
    }

    private SparkMax createFeederMotor() {
        var motor = new SparkMax(Constants.ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        
        config.smartCurrentLimit(Constants.ShooterConstants.FEEDER_CURRENT_LIMIT);
        config.voltageCompensation(12.0); 
            
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    private SparkMax createLauncherMotor() {
        var motor = new SparkMax(Constants.ShooterConstants.LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        
        config.inverted(true);
        config.smartCurrentLimit(Constants.ShooterConstants.LAUNCHER_CURRENT_LIMIT);
        config.voltageCompensation(12.0); 

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    public void setLauncherVoltage(double volts) {
        launcherMotor.setVoltage(volts);
    }

    public void setFeederVoltage(double volts) {
        feederMotor.setVoltage(volts);
    }

    public void stop() {
        launcherMotor.stopMotor();
        feederMotor.stopMotor();
    }
    
    // Actions based on Reference
    public void intake() {
        setFeederVoltage(SmartDashboard.getNumber(KEY_INTAKE_FEEDER, Constants.ShooterConstants.INTAKING_FEEDER_VOLTS));
        setLauncherVoltage(SmartDashboard.getNumber(KEY_INTAKE_LAUNCHER, Constants.ShooterConstants.INTAKING_LAUNCHER_VOLTS));
    }
    
    public void launch() {
        // Feeder voltage for launching
        setFeederVoltage(SmartDashboard.getNumber(KEY_LAUNCH_FEEDER, Constants.ShooterConstants.INTAKING_FEEDER_VOLTS));
        setLauncherVoltage(getTargetVolts());
    }
    
    public void spinUp() {
        setFeederVoltage(SmartDashboard.getNumber(KEY_SPINUP_FEEDER, Constants.ShooterConstants.IDLE_FEEDER_VOLTS));
        setLauncherVoltage(getTargetVolts());
    }

    public double getLauncherVelocity() {
        return launcherMotor.getEncoder().getVelocity();
    }
    
    public double getLauncherVoltage() {
        return launcherMotor.getAppliedOutput() * launcherMotor.getBusVoltage();
    }
    
    // Commands
    public Command intakeCommand() {
        return runEnd(this::intake, this::stop).withName("Intake");
    }

    public Command launchCommand() {
        return runEnd(this::launch, this::stop).withName("Launch");
    }
    
    // Kept for compatibility with Controls.java which expects a DoubleSupplier variant or similar
    public Command spinUpCommand(DoubleSupplier voltageSupplier) {
        return runEnd(() -> {
            setLauncherVoltage(voltageSupplier.getAsDouble());
            setFeederVoltage(SmartDashboard.getNumber(KEY_SPINUP_FEEDER, Constants.ShooterConstants.IDLE_FEEDER_VOLTS));
        }, this::stop).withName("SpinUp");
    }
    
    public Command spinUpCommand() {
        return runEnd(this::spinUp, this::stop).withName("SpinUp");
    }
    
    public Command spinUpAndShootCommand() {
        // Note: runEnd stops motors when finished, so chaining them needs care.
        // runEnd().andThen(runEnd()) works because the first one finishes (e.g. timeout) -> stop -> second starts.
        // But spinUpCommand needs to hold.
        // Actually, the reference implementation returns runEnd, which runs forever until interrupted.
        // To chain, we need a timeout or a condition.
        // We'll leave this simple logic for now.
        return spinUpCommand().withTimeout(1.0).andThen(launchCommand()).withName("SpinUpAndShoot");
    }
    
    public Command stopCommand() {
        return runOnce(this::stop).withName("Stop");
    }
    
    // Helpers for Logic
    public double getTargetVolts() {
         if (useDistanceTuning) {
             // If manual distance is set on dashboard, prefer it over auto?
             // Or let auto overwrite the dashboard value?
             // We'll read the dashboard value which might have been updated by setTargetDistance OR user
             double dist = SmartDashboard.getNumber("Shooter/Test Distance", currentDistance);
             return Constants.ShooterConstants.FuelVolts.get(dist);
         }
         return SmartDashboard.getNumber(KEY_LAUNCH_LAUNCHER, Constants.ShooterConstants.MANUAL_LAUNCHER_VOLTS);
    }

    public void setTargetDistance(Double distance) {
        if (distance != null) {
            this.currentDistance = distance;
            // Update the dashboard so the user sees what's happening
            SmartDashboard.putNumber("Shooter/Test Distance", distance);
        }
    }
    
    public boolean isLauncherAtSpeed() {
        double target = getTargetVolts();
        // Scaled check: Velocity should be proportional to Voltage.
        // Approx Kv check. 300 RPM/Volt is a very safe lower bound (NEO is ~470).
        // 4V * 300 = 1200 RPM.
        // 10V * 300 = 3000 RPM.
        if (Math.abs(target) > 2.0) {
            return Math.abs(getLauncherVelocity()) > (Math.abs(target) * 300.0);
        }
        return true; // Idle or low voltage, consider "ready"
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Launcher RPM", getLauncherVelocity());
        SmartDashboard.putNumber("Shooter/Feeder RPM", feederMotor.getEncoder().getVelocity());
        
        // Update flag from dashboard
        useDistanceTuning = SmartDashboard.getBoolean("Shooter/Use Distance Tuning", false);
        // Ensure the distance field exists
        SmartDashboard.setDefaultNumber("Shooter/Test Distance", currentDistance);
    }
}
