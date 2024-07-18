package frc.robot.subsystems.intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private CANSparkBase intakeMotor;
    private PIDController PIDController;

    public Intake(int intakeMotorId) {
        intakeMotor = new CANSparkMax(intakeMotorId, MotorType.kBrushless); 
        this.PIDController = new PIDController(IntakeConstants.kP, 
                                               IntakeConstants.kI, 
                                               IntakeConstants.kD);
    }


    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public void stopIntake(){
        intakeMotor.set(0);
    }

    public double getRawMotorRPM(){
        return intakeMotor.getEncoder().getVelocity();
    }

    public double getRPM(){
        return getRawMotorRPM() * IntakeConstants.kRPMConversionFactor;
    }

    public PIDController getPIDController(){
        return PIDController;
    }

    



    @Override
    public void periodic() {
    
    }
}