package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public class MotorControlGroup extends SubsystemBase {
    private List<CANSparkBase> motors;

    public MotorControlGroup(CANSparkBase... motors) {
        this.motors = new ArrayList<>();
        for (CANSparkBase motor : motors) {
            this.motors.add(motor);
        }
    }

    public void set(double power) {
        for (CANSparkBase motor : motors) {
            motor.set(power);
        }
    }

    public void set(double power, int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            motors.get(motorIndex).set(power);
        }
    }

    public void stop() {
        for (CANSparkBase motor : motors) {
            motor.set(0);
        }
    }

    public void stop(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            motors.get(motorIndex).set(0);
        }
    }

    public double getEncoderPosition(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            return motors.get(motorIndex).getEncoder().getPosition();
        }
        return 0;
    }

    public double getEncoderVelocity(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            return motors.get(motorIndex).getEncoder().getVelocity();
        }
        return 0;
    }

    public void setPID(double p, double i, double d) {
        for (CANSparkBase motor : motors) {
            motor.getPIDController().setP(p);
            motor.getPIDController().setI(i);
            motor.getPIDController().setD(d);
        }
    }

    public void setPID(double p, double i, double d, int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            motors.get(motorIndex).getPIDController().setP(p);
            motors.get(motorIndex).getPIDController().setI(i);
            motors.get(motorIndex).getPIDController().setD(d);
        }
    }

    public double getTemperature(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            return motors.get(motorIndex).getMotorTemperature();
        }
        return 0;
    }

    public double getAverageTemperature() {
        double totalTemp = 0;
        for (CANSparkBase motor : motors) {
            totalTemp += motor.getMotorTemperature();
        }
        return totalTemp / motors.size();
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        for (CANSparkBase motor : motors) {
            ((CANSparkMax) motor).setIdleMode(mode);
        }
    }

    public void setIdleMode(CANSparkMax.IdleMode mode, int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            ((CANSparkMax) motors.get(motorIndex)).setIdleMode(mode);
        }
    }

    public void burnFlash() {
        for (CANSparkBase motor : motors) {
            ((CANSparkMax) motor).burnFlash();
        }
    }

    public void burnFlash(int motorIndex) {
        if (motorIndex >= 0 && motorIndex < motors.size()) {
            ((CANSparkMax) motors.get(motorIndex)).burnFlash();
        }
    }
}
