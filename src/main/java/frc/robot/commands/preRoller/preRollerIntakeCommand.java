package frc.robot.commands.preRoller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.DIO.BeamBreak;
import frc.robot.subsystems.Shooter.preRoller;


public class preRollerIntakeCommand extends Command{
    
    private preRoller preRoller;
    private BeamBreak beamBreak;
    private Timer timer = new Timer(); // Create a Timer instance
    private boolean beamInitiallyBroken = false; // Flag to track if the beam has been broken
    private final double X = 5.0; // Duration in seconds after which the command should end


    public preRollerIntakeCommand(preRoller preRoller, BeamBreak beamBreak) {
        this.preRoller = preRoller;
        this.beamBreak = beamBreak;
        addRequirements(preRoller);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        timer.stop();
        timer.reset(); 
        preRoller.setPreRollerPower(1);
    }


    @Override
    public void execute() {
        if (!beamInitiallyBroken && beamBreak.isBroken()) {
            beamInitiallyBroken = true; // Set the flag when the beam is first broken
             // Start the timer
             timer.start();
        }
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return beamInitiallyBroken && timer.hasElapsed(X);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        preRoller.stopPreRoller();
        timer.stop();
        timer.reset();

    }
    
    
}
