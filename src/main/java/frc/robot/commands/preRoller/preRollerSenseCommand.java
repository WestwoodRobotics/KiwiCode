package frc.robot.commands.preRoller;
import java.util.LinkedList;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
        timer.stop();
        timer.reset();
        timer.stop();
        timer.start();
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

                try{
                    Thread.sleep(timerDelayMS);
                }
                catch(InterruptedException e){
                    e.printStackTrace();
                }
                
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        preRoller.stopPreRoller();
        timer.stop();
        timer.reset();
    }

    private void calculateRunningAverage(LinkedList<Double> currentList){
        double sum = 0;
        for(double current : currentList){
            sum += current;
        }
        runningAverage = sum / currentList.size();
    }


    



}
