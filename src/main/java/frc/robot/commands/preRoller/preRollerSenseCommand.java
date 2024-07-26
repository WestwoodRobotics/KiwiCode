package frc.robot.commands.preRoller;
import java.util.LinkedList;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.preRoller;

public class preRollerSenseCommand extends Command {

    private LinkedList<Double> currentList = new LinkedList<>();
    private PIDController preRollerPIDController;
    private long timerDelayMS;
    private double currentDeltaThreshold;
    private double runningAverage;
    private preRoller preRoller;
    private int sampleSize;
    private double targetRPM;
    private boolean sampleSizeReached = false;
    private boolean timerStarted = false;

    private Timer timer = new Timer();

    public preRollerSenseCommand(preRoller preRoller, double targetRPM, double timerDelay, double currentDeltaThreshold, int sampleSize) {
        this.timerDelayMS = (long)(timerDelay * 1000);
        this.currentDeltaThreshold = currentDeltaThreshold;
        this.preRoller = preRoller; 
        this.preRollerPIDController = preRoller.getPIDController();
        this.sampleSize = sampleSize;
        this.targetRPM = targetRPM;
        addRequirements(preRoller);
    }

    @Override
    public void initialize() {
        runningAverage = 0;
        currentList.clear();
        sampleSizeReached = false;
        timerStarted = false;
        timer.stop();
        timer.reset();
        preRollerPIDController.setSetpoint(targetRPM);
    }

    @Override
    public void execute() {

        currentList.add(preRoller.getOutputCurrent());
        if (currentList.size() > sampleSize) {
            currentList.removeFirst();
            sampleSizeReached = true;
        }
        preRoller.setPreRollerPower(preRollerPIDController.calculate(preRoller.getRawMotorRPM()));
    }

    @Override
    public boolean isFinished() {
        if (sampleSizeReached) {
            calculateRunningAverage(currentList);
            double currentDelta = Math.abs(preRoller.getOutputCurrent() - runningAverage);
            if (currentDelta > currentDeltaThreshold) {
                if (!timerStarted) {
                    timerStarted = true;
                    timer.reset();
                    timer.start();
                }
            }
            // Removed the else block that resets the timer if the current is within the threshold
            return timer.hasElapsed(timerDelayMS / 1000.0);
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        preRoller.stopPreRoller();
        preRoller.setHoldingNote(true);
        runningAverage = 0;
        currentList.clear();
        timerStarted = false;
        timer.stop();
        timer.reset();
        System.out.println("Finished Command");
    }

    private void calculateRunningAverage(LinkedList<Double> currentList) {
        double sum = 0;
        for (double current : currentList) {
            sum += current;
        }
        runningAverage = sum / currentList.size();
    }
}