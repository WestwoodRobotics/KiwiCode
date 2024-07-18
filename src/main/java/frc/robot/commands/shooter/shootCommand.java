package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.DIO.BeamBreak;
import frc.robot.subsystems.Shooter.TopBottomShooters;
import frc.robot.subsystems.Shooter.preRoller;

public class shootCommand extends Command {

    private TopBottomShooters topBottomShooters;
    private BeamBreak beamBreak;
    private preRoller preRoller;
    private boolean beamBreakInitiallyBroken = false;
    
    private Timer timer;

    public shootCommand(TopBottomShooters topBottomShooters, preRoller preRoller, BeamBreak beamBreak) {
        this.topBottomShooters = topBottomShooters;
        this.preRoller = preRoller;
        this.beamBreak = beamBreak;
        beamBreakInitiallyBroken = beamBreak.isBroken();
        addRequirements(topBottomShooters, preRoller);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        if (beamBreakInitiallyBroken) {
            timer.stop();
            timer.reset();
            topBottomShooters.setShooterPower(1);
            timer.start();
        }
        
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(2) && beamBreakInitiallyBroken){
            preRoller.setPreRollerPower(1);
        }
        
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(4) || !beamBreakInitiallyBroken;
    }

    @Override
    public void end(boolean interrupted) {
        topBottomShooters.stopShooter();
        preRoller.stopPreRoller();
        timer.stop();
        timer.reset();
    }

    
}
