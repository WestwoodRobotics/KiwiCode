package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;


public class preRoller extends SubsystemBase {
    // Define your member variables here
    private CANSparkMax preRollerMotor;

    private PIDController PIDController;

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

    @Override
    public void periodic() {
    
    }

}