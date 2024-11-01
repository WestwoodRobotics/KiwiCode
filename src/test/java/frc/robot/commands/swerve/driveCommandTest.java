package frc.robot.commands.swerve;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.SwerveDrive;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockitoAnnotations;
import edu.wpi.first.wpilibj.Timer;

public class driveCommandTest {

    private driveCommand command;
    private SwerveDrive mockSwerveDrive;
    private XboxController mockController;
    private Timer mockTimer;

    @BeforeEach
    public void setup() {
        MockitoAnnotations.openMocks(this);
        mockSwerveDrive = mock(SwerveDrive.class);
        mockController = mock(XboxController.class);
        mockTimer = mock(Timer.class);

        // Initialize the command with mocked subsystems
        command = new driveCommand(mockSwerveDrive, mockController);
        command.initialize();
    }

    @Test
    public void testDriveWithJoystickInputs() {
        // Set up joystick inputs
        when(mockController.getLeftX()).thenReturn(0.5);
        when(mockController.getLeftY()).thenReturn(-0.5);
        when(mockController.getRightX()).thenReturn(0.0);
        when(mockController.getBackButtonPressed()).thenReturn(false);

        // Simulate swerve drive state
        when(mockSwerveDrive.getSlowMode()).thenReturn(false);
        when(mockSwerveDrive.isYuMode()).thenReturn(false);
        when(mockSwerveDrive.getHeading()).thenReturn(0.0);

        // Mock the getModuleStates() method to return valid SwerveModuleState array
        SwerveModuleState[] mockStates = new SwerveModuleState[] {
            new SwerveModuleState(1.0, new Rotation2d(0)),
            new SwerveModuleState(1.0, new Rotation2d(0)),
            new SwerveModuleState(1.0, new Rotation2d(0)),
            new SwerveModuleState(1.0, new Rotation2d(0))
        };
        when(mockSwerveDrive.getModuleStates()).thenReturn(mockStates);

        // Execute the command
        command.execute();

        // Capture the arguments passed to drive method
        ArgumentCaptor<Double> xSpeedCaptor = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> ySpeedCaptor = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> rotCaptor = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Boolean> fieldRelativeCaptor = ArgumentCaptor.forClass(Boolean.class);
        ArgumentCaptor<Boolean> openLoopCaptor = ArgumentCaptor.forClass(Boolean.class);

        verify(mockSwerveDrive).drive(
                xSpeedCaptor.capture(),
                ySpeedCaptor.capture(),
                rotCaptor.capture(),
                fieldRelativeCaptor.capture(),
                openLoopCaptor.capture());

        // Calculate expected values
        double deadband = frc.robot.Constants.ControllerConstants.kDriveDeadband;
        double slowModeMultiplier = 1.0; // Since slowMode is false

        // Calculate similarityFactor as per driveCommand's execute method
        double desiredAngle = Math.atan2(-0.5, 0.5); // leftY=-0.5, leftX=0.5 => desiredAngle=-pi/4
        double similarityFactor = 0.0;
        for (SwerveModuleState state : mockStates) {
            double currentAngle = state.angle.getRadians();
            double angleDifference = desiredAngle - currentAngle;
            double cosineSimilarity = Math.abs(Math.cos(angleDifference));
            similarityFactor += cosineSimilarity;
        }
        similarityFactor /= mockStates.length; // Average similarity

        double expectedLeftX = -MathUtil.applyDeadband(0.5, deadband) * similarityFactor;
        double expectedLeftY = -MathUtil.applyDeadband(-0.5, deadband) * similarityFactor;
        double expectedRightX = -MathUtil.applyDeadband(0.0, deadband) * similarityFactor;

        // Verify the captured values
        assertEquals(expectedLeftX, xSpeedCaptor.getValue(), 1e-5, "Left X speed mismatch");
        assertEquals(expectedLeftY, ySpeedCaptor.getValue(), 1e-5, "Left Y speed mismatch");
        assertEquals(expectedRightX, rotCaptor.getValue(), 1e-5, "Rotation speed mismatch");
        assertTrue(fieldRelativeCaptor.getValue(), "Field relative should be true");
        assertFalse(openLoopCaptor.getValue(), "Open loop should be false");
    }

    @Test
    public void testDriveFullyForwardWithZeroModuleStates() {
        // Set up joystick inputs for full forward
        when(mockController.getLeftX()).thenReturn(0.0);
        when(mockController.getLeftY()).thenReturn(1.0);
        when(mockController.getRightX()).thenReturn(0.0);
        when(mockController.getBackButtonPressed()).thenReturn(false);
    
        // Simulate swerve drive state
        when(mockSwerveDrive.getSlowMode()).thenReturn(false);
        when(mockSwerveDrive.isYuMode()).thenReturn(false);
        when(mockSwerveDrive.getHeading()).thenReturn(0.0);
    
        // Mock the getModuleStates() method to return zero states
        SwerveModuleState[] mockStates = new SwerveModuleState[] {
            new SwerveModuleState(0.0, new Rotation2d(0)),
            new SwerveModuleState(0.0, new Rotation2d(0)),
            new SwerveModuleState(0.0, new Rotation2d(0)),
            new SwerveModuleState(0.0, new Rotation2d(0))
        };
        when(mockSwerveDrive.getModuleStates()).thenReturn(mockStates);
    
        // Execute the command
        command.execute();
    
        // Capture the arguments passed to drive method
        ArgumentCaptor<Double> xSpeedCaptor = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> ySpeedCaptor = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> rotCaptor = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Boolean> fieldRelativeCaptor = ArgumentCaptor.forClass(Boolean.class);
        ArgumentCaptor<Boolean> openLoopCaptor = ArgumentCaptor.forClass(Boolean.class);
    
        verify(mockSwerveDrive).drive(
                xSpeedCaptor.capture(),
                ySpeedCaptor.capture(),
                rotCaptor.capture(),
                fieldRelativeCaptor.capture(),
                openLoopCaptor.capture());
    
        // Calculate expected values
        double deadband = frc.robot.Constants.ControllerConstants.kDriveDeadband;
        double slowModeMultiplier = 1.0; // Since slowMode is false
    
        // When all modules are at zero speed, similarityFactor should be 1.0
        // This is handled by the allModulesStopped check in driveCommand
        double similarityFactor = 1.0;
    
        double expectedLeftX = -MathUtil.applyDeadband(0.0, deadband) * similarityFactor;
        double expectedLeftY = -MathUtil.applyDeadband(1.0, deadband) * similarityFactor;
        double expectedRightX = -MathUtil.applyDeadband(0.0, deadband) * similarityFactor;
    
        // Verify the captured values
        assertEquals(expectedLeftX, xSpeedCaptor.getValue(), 1e-5, "Left X speed mismatch");
        assertEquals(expectedLeftY, ySpeedCaptor.getValue(), 1e-5, "Left Y speed mismatch");
        assertEquals(expectedRightX, rotCaptor.getValue(), 1e-5, "Rotation speed mismatch");
        assertTrue(fieldRelativeCaptor.getValue(), "Field relative should be true");
        assertFalse(openLoopCaptor.getValue(), "Open loop should be false");
    }
    
    // Additional tests can be added here
}