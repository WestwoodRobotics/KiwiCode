package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;

/**
 * Command for setting the position of the intake shooter pivot.
 * This command uses a timer and a limit switch to determine when the pivot has reached the desired position.
 */
public class ampFreezeCommand extends Command{
    private AmpSubsystem ampSubsystem;

    private CANSparkMax intakePivotMotorController;

    private double currentAmpPose;



    /**
     * Constructs an IntakeShooterPosition command.
     * 
     * @param intakePivot The intake pivot subsystem.
     * @param position The target position for the intake shooter.
     */
    public ampFreezeCommand(AmpSubsystem ampSubsystem){
        this.ampSubsystem = ampSubsystem;

        addRequirements(ampSubsystem);
    }

    /**
     * Initializes the command by resetting and starting the timer.
     */
    @Override
    public void initialize(){
        currentAmpPose = ampSubsystem.getAmpEncoderPose();
        ampSubsystem.setAmpPosition(currentAmpPose);
    }

    /**
     * Executes the command by setting the pivot power based on the target position and current limit switch status.
     */
    @Override
    public void execute(){
        ampSubsystem.setPower(ampSubsystem.getCalculatedPIDPower());
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return false;
    }

    /**
     * Ends the command by stopping the pivot motor and setting the position state.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){
        
    }
}
