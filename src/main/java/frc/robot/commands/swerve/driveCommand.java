package frc.robot.commands.swerve;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The driveCommand class is responsible for controlling the swerve drive subsystem using an Xbox controller.
 * It handles joystick inputs to drive the robot in various directions and speeds, including a slow mode.
 */
public class driveCommand extends Command {

  private final SwerveDrive m_swerveDrive;
  private XboxController controller;
  private boolean slowMode;
  private boolean isYuMode;
  private PIDController rotationPIDController;
  private double targetHeading;
  private boolean isRotInput;
  private Timer timer;

  /**
   * Constructs a new driveCommand.
   * 
   * @param swerveDrive The swerve drive subsystem used by this command.
   * @param controller The Xbox controller used to control the swerve drive.
   */
  public driveCommand(SwerveDrive swerveDrive, XboxController controller) {
    m_swerveDrive = swerveDrive;
    this.controller = controller;
    timer = new Timer();
    addRequirements(swerveDrive);
  }

  /**
   * Initializes the command by setting the slow mode to false.
   */
  @Override
  public void initialize() {
    slowMode = false;
    isRotInput = true;
    rotationPIDController = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
    rotationPIDController.setTolerance(5);
    targetHeading = m_swerveDrive.getHeading();
    rotationPIDController.setSetpoint(targetHeading);
  }

  /**
   * Executes the command by reading joystick inputs and driving the swerve drive subsystem.
   * It handles slow mode and Yu mode toggling based on controller inputs.
   */

  @Override
  public void execute() {
    slowMode = m_swerveDrive.getSlowMode();
    isYuMode = m_swerveDrive.isYuMode();

    if (controller.getBackButtonPressed()) {
        slowMode = !slowMode;
    }

    double leftX, leftY, rightX;

    if (!isYuMode) {
        leftX = -MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDriveDeadband);
        leftY = -MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDriveDeadband);
        rightX = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDriveDeadband);
    } else {
        leftX = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDriveDeadband);
        leftY = -MathUtil.applyDeadband(controller.getRightY(), ControllerConstants.kDriveDeadband);
        rightX = -MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDriveDeadband);
    }

    if (slowMode) {
        double multiplier = Constants.DriveConstants.slowModeMultiplier;
        leftX *= multiplier;
        leftY *= multiplier;
        rightX *= multiplier;
    }

    // Handle rotation input and PID control (existing code remains)

    // Calculate desired drive vector
    Translation2d desiredDriveVector = new Translation2d(leftX, leftY);
    double desiredAngle = Math.atan2(desiredDriveVector.getY(), desiredDriveVector.getX());
    double desiredSpeed = desiredDriveVector.getNorm();

    // Get current module states
    SwerveModuleState[] currentStates = m_swerveDrive.getModuleStates();

    // Calculate similarity factor
    double similarityFactor = 0.0;
    for (SwerveModuleState state : currentStates) {
        double currentAngle = state.angle.getRadians();
        double angleDifference = desiredAngle - currentAngle;
        double cosineSimilarity = Math.abs(Math.cos(angleDifference));
        similarityFactor += cosineSimilarity;
    }
    similarityFactor /= currentStates.length; // Average similarity

    // Handle edge case where all module speeds are zero
    boolean allModulesStopped = Arrays.stream(currentStates)
                                    .allMatch(state -> state.speedMetersPerSecond == 0.0);
    if (allModulesStopped) {
        similarityFactor = 1.0;
    }

    // **Set a minimum similarity factor to avoid zero power**
    similarityFactor = Math.max(similarityFactor, 0.1); // Set a minimum threshold (e.g., 0.1)

    // Adjust desired speed based on similarity factor
    double adjustedSpeed = desiredSpeed * similarityFactor;

    // Reconstruct adjusted drive vector with adjusted speed
    double adjustedLeftX = adjustedSpeed * Math.cos(desiredAngle);
    double adjustedLeftY = adjustedSpeed * Math.sin(desiredAngle);

    // Drive the robot with adjusted inputs
    m_swerveDrive.drive(adjustedLeftX, adjustedLeftY, rightX, true, false);
  }
  /**
   * Ends the command. This method is called once when the command ends or is interrupted.
   * 
   * @param interrupted Whether the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Toggles the Yu mode.
   */
  public void toggleYuMode() {
    isYuMode = !isYuMode;
  }

  /**
   * Returns whether the command has finished.
   * 
   * @return Always returns false, as this command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
