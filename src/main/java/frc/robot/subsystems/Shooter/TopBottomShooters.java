package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TopBottomShooters extends SubsystemBase {

    private CANSparkFlex topRoller;
    private CANSparkFlex bottomRoller;

    private boolean isPIDCOntrol;

    private PIDController TopRollerPIDController;
    private PIDController BottomRollerPIDController;

    private double TopRollerRPMSetpoint;

    private double BottomRollerRPMSetpoint;

    public TopBottomShooters(int topRollerId, int bottomRollerId, boolean isPIDControl) {
        topRoller = new CANSparkFlex(topRollerId, MotorType.kBrushless);
        bottomRoller = new CANSparkFlex(bottomRollerId, MotorType.kBrushless);
        this.isPIDCOntrol = isPIDControl;

        this.TopRollerPIDController = new PIDController(ShooterConstants.kTopRollerP, 
                                                        ShooterConstants.kTopRollerI, 
                                                        ShooterConstants.kTopRollerD);

        this.BottomRollerPIDController = new PIDController(ShooterConstants.kBottomRollerP, 
                                                           ShooterConstants.kBottomRollerI, 
                                                           ShooterConstants.kBottomRollerD);
    }

    public void setTopRollerPower(double power) {
        topRoller.set(power);
    }

    public void setBottomRollerPower(double power) {
        bottomRoller.set(power);
    }

    public void setShooterPower(double power) {
        setTopRollerPower(power);
        setBottomRollerPower(power);
    }

    public double getTopRollerMotorRawRPM() {
        return topRoller.getEncoder().getVelocity();
    }

    public double getTopRollerRPM() {
        return getTopRollerMotorRawRPM() * ShooterConstants.kTopRollerRPMConversionFactor;
    }

    public double getBottomRollerMotorRawRPM() {
        return bottomRoller.getEncoder().getVelocity();
    }

    public double getBottomRollerRPM() {
        return getBottomRollerMotorRawRPM() * ShooterConstants.kBottomRollerRPMConversionFactor;
    }

    public PIDController getTopRollerPIDController() {
        return TopRollerPIDController;
    }

    public PIDController getBottomRollerPIDController() {
        return BottomRollerPIDController;
    }
    
    public void setTopRollerRPMSetpoint(double setpoint) {
        TopRollerRPMSetpoint = setpoint;
    }

    public void setBottomRollerRPMSetpoint(double setpoint) {
        BottomRollerRPMSetpoint = setpoint;
    }

    public double getTopRollerRPMSetpoint() {
        return TopRollerRPMSetpoint;
    }

    public double getBottomRollerRPMSetpoint() {
        return BottomRollerRPMSetpoint;
    }

    public boolean isPIDControl() {
        return isPIDCOntrol;
    }
    
    public void stopShooter() {
        setTopRollerPower(0);
        setBottomRollerPower(0);
    }
    
    @Override
    public void periodic() {
    
    }


}
