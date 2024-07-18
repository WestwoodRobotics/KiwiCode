package frc.robot.commands.preRoller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.DIO.BeamBreak;
import frc.robot.subsystems.Shooter.preRoller;


public class preRollerSpitCommand extends Command{
    
    private preRoller preRoller;


    public preRollerSpitCommand(preRoller preRoller) {
        this.preRoller = preRoller;
        addRequirements(preRoller);
    }

    @Override
    public void initialize() {
        preRoller.setPreRollerPower(-1);
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
        // TODO Auto-generated method stub
        preRoller.stopPreRoller();
    }
    
    
}
