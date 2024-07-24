package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UtilityConstants;
import edu.wpi.first.math.controller.PIDController;


public class preRoller extends SubsystemBase {
    // Define your member variables here
    private CANSparkMax preRollerMotor;

    private PIDController PIDController;
    private double currentCurrentOffset = 0;

    private boolean printed = false;
    private double lastCurrent;

    private boolean isHoldingNote;

    // Constructor
    public preRoller() {
        preRollerMotor = new CANSparkMax(ShooterConstants.kPreRollerPort, MotorType.kBrushless);
        this.PIDController = new PIDController(ShooterConstants.kPreRollerP, 
                                               ShooterConstants.kPreRollerI, 
                                               ShooterConstants.kPreRollerD);
    }

    // Define your methods here

    public void setPreRollerPower(double power) {
        preRollerMotor.set(power);
    }

    public void stopPreRoller(){
        preRollerMotor.set(0);
    }

    public double getRawMotorRPM(){
        return preRollerMotor.getEncoder().getVelocity();
    }

    public double getRPM(){
        return getRawMotorRPM() * ShooterConstants.kPreRollerRPMConversionFactor;
    }

    public PIDController getPIDController(){
        return PIDController;
    }

    //A method that will return the current output current of the motor
    public double getOutputCurrent(){
        return preRollerMotor.getOutputCurrent() - currentCurrentOffset;
    }
    

    @Override
    public void periodic() {
        if (UtilityConstants.debugMode){
            if(!printed){
                System.out.println("PreRoller Current: " + getOutputCurrent());
                lastCurrent = this.getOutputCurrent();
                printed = true;
            }
            else if(Math.abs(lastCurrent - preRollerMotor.getOutputCurrent()) > 10){
                System.out.println("PreRoller Current: " + getOutputCurrent());
                lastCurrent = this.getOutputCurrent();
            }
        }
    }

    public void resetOutputCurrent(){
        currentCurrentOffset = getOutputCurrent();
    }

    public void setHoldingNote(boolean isHoldingNote, Class<?> caller){
        if ((caller.getClass().getName().equals("frc.robot.commands.preRoller.preRollerSenseCommand"))){
            this.isHoldingNote = isHoldingNote;
        } else {
            throw new IllegalArgumentException("The caller must be either preRollerSenseCommand or shooterSenseCommand or intakeSenseCommand");
        }
    }

    public boolean getHoldingNote(Class<?> caller){
        if ((caller.getClass().getName().equals("frc.robot.commands.preRoller.preRollerSenseCommand")) 
            || (caller.getClass().getName().equals("frc.robot.commands.shooter.shooterSenseCommand"))
            || (caller.getClass().getName().equals("frc.robot.commands.intake.intakeSenseCommand"))){
            return isHoldingNote;
        } else {
            throw new IllegalArgumentException("The caller must be either preRollerSenseCommand or shooterSenseCommand or intakeSenseCommand");
    }

    

    
    }
}