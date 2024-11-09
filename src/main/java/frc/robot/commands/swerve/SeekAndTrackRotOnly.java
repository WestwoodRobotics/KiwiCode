package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.LimelightHelpers;

public class SeekAndTrackRotOnly extends Command {
    private final SwerveDrive swerve;
    private final String limelightName;
    private final PIDController rotationPID;
    private final double SEARCH_ROTATION_SPEED = 0.3; // Adjust this for search speed

    
    public SeekAndTrackRotOnly(SwerveDrive swerve, String limelightName) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        
        // Tune PID values for your robot
        rotationPID = new PIDController(0.015, 0.0, 0.001);
        rotationPID.setTolerance(1.0); // 1 degree tolerance
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        rotationPID.reset();

    }

    @Override
    public void execute() {
        // Check if we can see a target
        if (!LimelightHelpers.getTV(limelightName)) {

            // No target visible - keep searching by rotating
            swerve.drive(0, 0, SEARCH_ROTATION_SPEED, true, false);
            return;
        }

        // We see a target

        
        // Get horizontal offset from target
        double tx = LimelightHelpers.getTX(limelightName);
        
        // Calculate rotation to track target
        double rotationOutput = rotationPID.calculate(tx, 0.0);
        
        // Apply rotation only - no translation
        swerve.drive(0, 0, -rotationOutput, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - must be interrupted
        return false;
    }
}