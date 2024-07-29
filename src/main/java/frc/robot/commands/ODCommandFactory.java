package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.intakePIDCommand;
import frc.robot.commands.preRoller.preRollerSenseCommand;
import frc.robot.commands.shooter.shooterPIDCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.preRoller;
import frc.robot.subsystems.intake.Intake;

public class ODCommandFactory {
    private final Intake m_intake;
    private final preRoller m_preRoller;
    private final Shooter m_shooter;

    public ODCommandFactory(Intake m_intake, preRoller m_preRoller, Shooter m_shooter){
        this.m_intake = m_intake;
        this.m_preRoller = m_preRoller;
        this.m_shooter = m_shooter;
    }

    public Command intakeSenseCommand(){
        return new preRollerSenseCommand(m_preRoller, 6180, 0.1, 20, 30).alongWith(new InstantCommand(()->m_intake.setIntakePower(0.8)));
    }

    public Command stopIntakeSenseCommand(){
        return new InstantCommand(() -> m_intake.stopIntake(), m_intake).alongWith(new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller));
    }
    

    public Command revUpAndShootCommand(double power, double targetRPM){
        return new shooterPIDCommand(m_shooter, power, targetRPM)
        .andThen(new InstantCommand(() -> m_preRoller.setPreRollerPower(0.7)))
        .andThen(new WaitCommand(0.6))
        .andThen(new InstantCommand(()-> m_shooter.setShooterPower(40)).alongWith(new InstantCommand(() -> m_preRoller.stopPreRoller())));
    }


    public Command stopShooterCommand(){
        //return new InstantCommand(() -> m_shooter.stopShooter()).andThen(new InstantCommand(() -> m_preRoller.stopPreRoller()));
        return new InstantCommand(() -> m_shooter.stopShooter(), m_shooter).andThen(new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller)); 
    }

    public Command stopPreRollerCommand(){
        return new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller);
    }

    public Command stopIntakeCommand(){
        return new InstantCommand(() -> m_intake.stopIntake(), m_intake);
    }

    public Command revUpShooter(){
        return new InstantCommand(() -> m_shooter.setShooterPower(0.8));
    }

    public Command fireNote(){
        return new InstantCommand(() -> m_preRoller.setPreRollerPower(0.5)).andThen(new WaitCommand(0.75)).andThen(new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller));
    }

    public Command stopAllCommand(){
        return new InstantCommand(() -> m_shooter.stopShooter(), m_shooter).alongWith(this.stopIntakeSenseCommand());
    }

}
