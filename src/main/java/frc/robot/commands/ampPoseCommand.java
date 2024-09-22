package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AmpSubsystem;

public class ampPoseCommand extends CommandBase {
    private final AmpSubsystem ampSubsystem;
    private final double targetPose;
    private double currentPose;

    public ampPoseCommand(AmpSubsystem ampSubsystem, double targetPose) {
        this.ampSubsystem = ampSubsystem;
        this.targetPose = targetPose;
        addRequirements(ampSubsystem);
    }

    @Override
    public void initialize() {
        ampSubsystem.setAmpPosition(targetPose);
    }

    @Override
    public void execute() {
        ampSubsystem.setPower(ampSubsystem.getCalculatedPIDPower());
    }

    @Override
    public boolean isFinished() {
        return ampSubsystem.isAtPose();
    }

    @Override
    public void end(boolean interrupted) {
        ampSubsystem.setPower(0);
    }
}
