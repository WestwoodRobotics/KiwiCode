package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class shooterPIDCommand extends Command{
    private Shooter m_shooter;
    private double targetRPM;
    private double power;
    private PIDController TopMotorPIDController;
    private PIDController BottomMotorPIDController;

    public shooterPIDCommand(Shooter m_shooter, double power, double targetRPM){
        this.m_shooter = m_shooter;
        this.power = power;
        TopMotorPIDController = m_shooter.getTopRollerPIDController();
        BottomMotorPIDController = m_shooter.getBottomRollerPIDController();
        this.targetRPM = targetRPM;
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter.setShooterPower(power);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()){
            return (this.m_shooter.getTopRollerMotorRawRPM() >= targetRPM);
        }
        return false;
        
    }
    
    
}
