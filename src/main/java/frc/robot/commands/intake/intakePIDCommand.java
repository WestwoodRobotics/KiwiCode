package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class intakePIDCommand extends Command{

    private double targetRPM;
    private Intake intake;
    private PIDController intakePIDController;
    

    public intakePIDCommand(Intake intake, double targetRPM){
        this.intake = intake;
        this.targetRPM = targetRPM;
        this.intakePIDController = intake.getPIDController();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intakePIDController.setSetpoint(targetRPM);
    }

    @Override
    public void execute() {
        intake.setIntakePower(intakePIDController.calculate(intake.getRawMotorRPM()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    




    
}
