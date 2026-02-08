package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A Command that performs a Relay Auto-Tuning process for the Rotation PID.
 * It oscillates the robot between +/- setpoints to measure natural frequency and amplitude.
 * Based on the Astrom-Hagglund Relay method (simplified for FRC).
 * 
 * Calculates kP, kI, kD using Ziegler-Nichols and Tyreus-Luyben methods.
 */
public class AutoTuneRotation extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Timer timer = new Timer();
    
    
    private double relayHeight = 0.2; 
    private double hysteresis = 2.0; 
    private double targetAngle;
    
    private double peakAmplitude = 0.0;
    private double lastCrossingTime = 0.0;
    private int cycleCount = 0;
    private double totalPeriod = 0.0;
    private double lastError = 0.0;
    
    public AutoTuneRotation(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        SmartDashboard.putNumber("AutoTune/Relay Speed", relayHeight);
        SmartDashboard.putNumber("AutoTune/Hysteresis Deg", hysteresis);
    }

    @Override
    public void initialize() {
        relayHeight = SmartDashboard.getNumber("AutoTune/Relay Speed", 0.2);
        hysteresis = SmartDashboard.getNumber("AutoTune/Hysteresis Deg", 2.0);
        
        targetAngle = drivetrain.getState().Pose.getRotation().getDegrees();
        peakAmplitude = 0.0;
        cycleCount = 0;
        totalPeriod = 0.0;
        lastError = 0.0;
        
        timer.restart();
        lastCrossingTime = timer.get();
        
        System.out.println("AutoTuner Started. Target: " + targetAngle);
    }

    @Override
    public void execute() {
        double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
        
        double error = Rotation2d.fromDegrees(targetAngle - currentAngle).getDegrees();
        
        
        double output = 0.0;
        if (error > hysteresis) {
            output = -relayHeight; 
        } else if (error < -hysteresis) {
            output = relayHeight; 
        } else {
            
            output = error > 0 ? -relayHeight : relayHeight;
        }
        
        
        peakAmplitude = Math.max(peakAmplitude, Math.abs(error));
        
        
        
        if (Math.signum(error) != Math.signum(lastError) && Math.abs(error) < 5.0) {
            double now = timer.get();
            double halfPeriod = now - lastCrossingTime;
            
            if (halfPeriod > 0.1) { 
                totalPeriod += (halfPeriod * 2.0);
                cycleCount++;
                lastCrossingTime = now;
            }
        }
        lastError = error;
        
        
        double maxRate = 2 * Math.PI; 
        var driveRequest = new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric()
            .withRotationalRate(output * maxRate);
            
        drivetrain.setControl(driveRequest);
        
        SmartDashboard.putNumber("AutoTune/Error", error);
        SmartDashboard.putNumber("AutoTune/Output", output);
        SmartDashboard.putNumber("AutoTune/Cycles", cycleCount);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake());
        
        if (cycleCount < 3) {
            System.out.println("Not enough cycles to tune. Run longer.");
            return;
        }

        double avgTu = totalPeriod / cycleCount; 
        double d = relayHeight; 
        double a = peakAmplitude; 
        
        
        
        
        
        double Ku = (4.0 * d) / (Math.PI * a);
        
        
        
        double Kp_ZN = 0.6 * Ku;
        double Ki_ZN = 1.2 * Ku / avgTu;
        double Kd_ZN = 0.075 * Ku * avgTu;
        
        
        
        double Kp_TL = 0.45 * Ku; 
        double Ti_TL = 2.2 * avgTu;
        double Ki_TL = Kp_TL / Ti_TL; 
        double Td_TL = avgTu / 6.3;
        double Kd_TL = Kp_TL * Td_TL;
        
        
        double Kp_NO = 0.2 * Ku;
        double Ki_NO = 0.4 * Ku / avgTu;
        double Kd_NO = 0.066 * Ku * avgTu;

        
        SmartDashboard.putNumber("AutoTune/Result_Ku", Ku);
        SmartDashboard.putNumber("AutoTune/Result_Tu", avgTu);
        
        SmartDashboard.putNumber("AutoTune/ZN_kP", Kp_ZN);
        SmartDashboard.putNumber("AutoTune/ZN_kI", Ki_ZN);
        SmartDashboard.putNumber("AutoTune/ZN_kD", Kd_ZN);
        
        SmartDashboard.putNumber("AutoTune/TL_kP", Kp_TL);
        SmartDashboard.putNumber("AutoTune/TL_kI", Ki_TL);
        SmartDashboard.putNumber("AutoTune/TL_kD", Kd_TL);
        
        System.out.println("AutoTune Finished.");
        System.out.printf("Ku: %.5f, Tu: %.3f\n", Ku, avgTu);
        System.out.printf("ZN -> kP: %.5f, kI: %.5f, kD: %.5f\n", Kp_ZN, Ki_ZN, Kd_ZN);
        System.out.printf("TL -> kP: %.5f, kI: %.5f, kD: %.5f\n", Kp_TL, Ki_TL, Kd_TL);
    }
}
