package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.DIO.BeamBreak;
import frc.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj.Timer; // Import the Timer class

public class UTBIntakeCommand extends Command {
    private Intake intake;
    private BeamBreak beamBreak;
    private Timer timer = new Timer(); // Create a Timer instance
    private boolean beamInitiallyBroken = false; // Flag to track if the beam has been broken
    private final double X = 5.0; // Duration in seconds after which the command should end

    public UTBIntakeCommand(Intake intake, BeamBreak beamBreak) {
        this.intake = intake;
        this.beamBreak = beamBreak;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        intake.setIntakePower(1);
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
        // Return true if the timer has been running for more than X seconds since the beam was broken
        return beamInitiallyBroken && timer.hasElapsed(X);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        timer.stop();
        timer.reset();
    }
}