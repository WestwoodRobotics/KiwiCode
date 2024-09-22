package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;

public class AmpSubsystem extends SubsystemBase {
    private final CANSparkMax AmpMotor;
    private final PIDController ampPIDController;

    public AmpSubsystem(int motorPort) {
        AmpMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
        ampPIDController = new PIDController(AmpConstants.kP, AmpConstants.kI, AmpConstants.kD);
        ampPIDController.setTolerance(0.2);
    }

    public void setPower(double power) {
        AmpMotor.set(power);
    }

    public void setAmpPosition(double setPoint) {
        ampPIDController.setSetpoint(setPoint);
    }

    public double getSetpoint() {
        return ampPIDController.getSetpoint();
    }

    public boolean isAtPose() {
        return ampPIDController.atSetpoint();
    }

    public double getAmpEncoderPose() {
        return AmpMotor.getEncoder().getPosition();
    }

    public double getCalculatedPIDPower() {
        return ampPIDController.calculate(getAmpEncoderPose());
    }
}
