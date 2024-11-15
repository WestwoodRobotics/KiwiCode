package frc.robot.subsystems.axe;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxeConstants;

public class Axe extends SubsystemBase {

    private CANSparkFlex axeMotor;
    private PIDController axePIDController;

    public Axe() {
        this.axeMotor = new CANSparkFlex(AxeConstants.kAxeMotorPort, MotorType.kBrushless);
        this.axePIDController = new PIDController(AxeConstants.kP, AxeConstants.kI, AxeConstants.kD);
    }

    public void setAxePower(double power) {
        axeMotor.set(power);
    }

    public double getAxePosition() {
        return axeMotor.getEncoder().getPosition();
    }

    public PIDController getPIDController() {
        return axePIDController;
    }
    
}
