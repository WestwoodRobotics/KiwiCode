package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

/**
 * The shooterPIDCommand class is responsible for controlling the shooter subsystem using a PID controller.
 * It sets the shooter motor power based on the target RPM and the current RPM of the motor.
 */
public class shooterPIDCommand extends Command{
    private Shooter m_shooter;
    private double targetRPM;
    private double power;
    private PIDController TopMotorPIDController;
    private PIDController BottomMotorPIDController;

    /**
     * Constructs a new shooterPIDCommand.
     * 
     * @param m_shooter The shooter subsystem used by this command.
     * @param power The power to set for the shooter motor.
     * @param targetRPM The target RPM for the shooter motor.
     */
    public shooterPIDCommand(Shooter m_shooter, double power, double targetRPM){
        this.m_shooter = m_shooter;
        this.power = power;
        TopMotorPIDController = m_shooter.getTopRollerPIDController();
        BottomMotorPIDController = m_shooter.getBottomRollerPIDController();
        this.targetRPM = targetRPM;
        addRequirements(m_shooter);
    }

    /**
     * Initializes the command by setting the shooter motor power.
     */
    @Override
    public void initialize() {
        m_shooter.setShooterPower(power);
    }

    /**
     * Executes the command. This method is called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
    }

    /**
     * Ends the command. This method is called once when the command ends or is interrupted.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return True if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()){
            return (this.m_shooter.getTopRollerMotorRawRPM() >= targetRPM);
        }
        return false;
        
    }
    
    
}
