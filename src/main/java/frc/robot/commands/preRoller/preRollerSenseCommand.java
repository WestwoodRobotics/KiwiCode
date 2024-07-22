package frc.robot.commands.preRoller;
import java.util.LinkedList;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter.preRoller;

public class preRollerSenseCommand extends Command{

    private LinkedList<Double> currentList = new LinkedList<>();
    private long timerDelayMS;
    private double currentDeltaThreshold;
    private double runningAverage;
    private preRoller preRoller;
    private int sampleSize;
    private double preRollerPower;
    private boolean sampleSizeReached = false;
    private boolean timerStarted = false;
    private Timer timer = new Timer();


    public preRollerSenseCommand(preRoller preRoller, double preRollerPower, double timerDelay, double currentDeltaThreshold, int sampleSize) {
        this.timerDelayMS = (long)(timerDelay * 1000);
        
        this.currentDeltaThreshold = currentDeltaThreshold;
        this.preRoller = preRoller;
        this.sampleSize = sampleSize;
        this.preRollerPower = preRollerPower;
        addRequirements(preRoller);
    }

    @Override
    public void initialize() {
        runningAverage = 0;
        preRoller.setPreRollerPower(preRollerPower);
    }

    @Override
    public void execute() {
        currentList.add(preRoller.getOutputCurrent());
        if(currentList.size() > sampleSize){
            currentList.removeFirst();
            sampleSizeReached = true;
        }

    }

    @Override
    public boolean isFinished() {
        if(sampleSizeReached){
            calculateRunningAverage(currentList);
            double currentDelta = Math.abs(preRoller.getOutputCurrent() - runningAverage);
            if((currentDelta > currentDeltaThreshold)){
                if(!timerStarted){
                    timerStarted = true;
                    timer.stop();
                    timer.reset();
                    timer.start();
                }
                return timer.hasElapsed(timerDelayMS/1000);

            }
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        preRoller.stopPreRoller();
        runningAverage = 0;
        currentList.clear();
        timerStarted = false;
        timer.stop();
        timer.reset();
        timer.stop();
        currentDeltaThreshold = 0;
        timerDelayMS = 0;
        sampleSize = 0;
        System.out.println("Finished Command");
    }

    private void calculateRunningAverage(LinkedList<Double> currentList){
        double sum = 0;
        for(double current : currentList){
            sum += current;
        }
        runningAverage = sum / currentList.size();
    }


    



}
