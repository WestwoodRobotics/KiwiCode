package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.DIO.BeamBreak;
import frc.robot.subsystems.Shooter.preRoller;
import frc.robot.subsystems.intake.Intake;


public class UTBSpitCommand extends Command{
    
    private Intake intake;


    public UTBSpitCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakePower(-1.0);
    }


    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
    
    
}
