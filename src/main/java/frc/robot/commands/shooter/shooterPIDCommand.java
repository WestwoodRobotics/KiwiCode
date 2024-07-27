package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class shooterPIDCommand extends Command{
    private Shooter m_shooter;
    private double targetRPM;
    private PIDController TopMotorPIDController;
    private PIDController BottomMotorPIDController;

    public shooterPIDCommand(Shooter m_shooter, double targetRPM){
        this.m_shooter = m_shooter;
        TopMotorPIDController = m_shooter.getTopRollerPIDController();
        BottomMotorPIDController = m_shooter.getBottomRollerPIDController();
        this.targetRPM = targetRPM;
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TopMotorPIDController.setSetpoint(targetRPM);
        BottomMotorPIDController.setSetpoint(targetRPM);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.setTopRollerPower(TopMotorPIDController.calculate(m_shooter.getTopRollerMotorRawRPM()));
        m_shooter.setBottomRollerPower(BottomMotorPIDController.calculate(m_shooter.getBottomRollerMotorRawRPM()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()){
            return false;
        }
        return false;
        
    }
    
    
}
