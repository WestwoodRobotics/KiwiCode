
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.ODCommandFactory;
import frc.robot.commands.intake.UTBIntakeCommand;
import frc.robot.commands.preRoller.preRollerIntakeCommand;
import frc.robot.commands.preRoller.preRollerSenseCommand;
import frc.robot.commands.swerve.driveCommand;
import frc.robot.subsystems.Shooter.TopBottomShooters;
import frc.robot.subsystems.Shooter.preRoller;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveDrive m_robotDrive = new SwerveDrive();
  private final Intake m_intake = new Intake();
  private final preRoller m_preRoller = new preRoller();
  private final TopBottomShooters m_shooter = new TopBottomShooters(false);
  private final SendableChooser<Command> autoChooser;


  // LED for indicating robot state, not implemented in hardware.

  // The driver's controller
  XboxController m_driverController = new XboxController(PortConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(PortConstants.kOperatorControllerPort);

  private final JoystickButton DriverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton DriverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton DriverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton DriverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton DriverGyroButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton OperatorStartButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);


  private final POVButton DriverDPadUp = new POVButton(m_driverController, 0);
  private final POVButton DriverDPadRight = new POVButton(m_driverController, 90);
  private final POVButton DriverDPadDown = new POVButton(m_driverController, 180);
  private final POVButton DriverDPadLeft = new POVButton(m_driverController, 270);

  private final JoystickButton DriverRightBumper = new JoystickButton(m_driverController,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton DriverLeftBumper = new JoystickButton(m_driverController,
      XboxController.Button.kLeftBumper.value);
      
  private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
  private final Trigger driverRightTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);

  private final JoystickButton OperatorAButton = new JoystickButton(m_operatorController,
      XboxController.Button.kA.value);
  private final JoystickButton OperatorBButton = new JoystickButton(m_operatorController,
      XboxController.Button.kB.value);
  private final JoystickButton OperatorXButton = new JoystickButton(m_operatorController,
      XboxController.Button.kX.value);
  private final JoystickButton OperatorYButton = new JoystickButton(m_operatorController,
      XboxController.Button.kY.value);

  private final POVButton OperatorDPadUp = new POVButton(m_operatorController, 0);
  private final POVButton OperatorDPadRight = new POVButton(m_operatorController, 90);
  private final POVButton OperatorDPadDown = new POVButton(m_operatorController, 180);
  private final POVButton OperatorDPadLeft = new POVButton(m_operatorController, 270);

  private Trigger operatorRightYTrigger = new Trigger(() -> Math.abs(m_operatorController.getRightY()) > 0.10);

  private final Trigger operatorLeftTrigger = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.5);
  private final Trigger operatorRightTrigger = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5);

  private final JoystickButton OperatorRightBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton OperatorLeftBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kLeftBumper.value);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private ODCommandFactory ODCommandFactory = new ODCommandFactory(m_intake, m_preRoller, m_shooter);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("SpinSensePreRoller", ODCommandFactory.intakeSenseCommand());
    NamedCommands.registerCommand("Intake", ODCommandFactory.intakeSenseCommand());
    NamedCommands.registerCommand("StopIntake", ODCommandFactory.stopIntakeSenseCommand());
    NamedCommands.registerCommand("RevUpAndShoot", ODCommandFactory.revUpAndShootCommand(2));
    NamedCommands.registerCommand("StopShooter", ODCommandFactory.stopShooterCommand());
    NamedCommands.registerCommand("releasePreRollerCommand", ODCommandFactory.fireNote());
    NamedCommands.registerCommand("stopAllCommand", ODCommandFactory.stopAllCommand());

    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Configure default commands 
    // m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /*
   * Use this method to define your button->command mappings. Buttons can be
   * created by 
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  


private void configureButtonBindings() {
    /*
     * DRIVER BUTTON MAPPINGS
     */

    DriverGyroButton.whileTrue(new InstantCommand(
        () -> m_robotDrive.resetGyro(),
        m_robotDrive));

    driverLeftTrigger.onTrue(ODCommandFactory.intakeSenseCommand());
    driverLeftTrigger.onFalse(ODCommandFactory.stopIntakeSenseCommand());

    driverRightTrigger.onTrue(ODCommandFactory.revUpShooter());
    driverRightTrigger.onFalse(ODCommandFactory.stopShooterCommand());

    DriverRightBumper.onTrue(ODCommandFactory.revUpAndShootCommand(2));
    DriverRightBumper.onFalse(ODCommandFactory.stopShooterCommand());


    DriverDPadDown.onTrue(new InstantCommand(() -> m_shooter.setShooterPower(-0.85), m_shooter));

    // DriverBButton.onTrue(new InstantCommand(() -> m_preRoller.setPreRollerPower(1), m_preRoller));
    // DriverBButton.onFalse(new InstantCommand(() -> m_preRoller.setPreRollerPower(0), m_preRoller));
    DriverBButton.onTrue(ODCommandFactory.intakeSenseCommand());
    DriverBButton.onFalse(ODCommandFactory.stopPreRollerCommand().alongWith(ODCommandFactory.stopIntakeCommand()));
    DriverXButton.onTrue(new InstantCommand(() -> m_preRoller.setPreRollerPower(-1), m_preRoller));
    DriverXButton.onFalse(new InstantCommand(() -> m_preRoller.setPreRollerPower(0), m_preRoller));

    DriverAButton.onTrue(new InstantCommand(() -> m_intake.setIntakePower(1), m_intake));
    DriverAButton.onFalse(new InstantCommand(() -> m_intake.setIntakePower(0), m_intake));
    DriverYButton.onTrue(new InstantCommand(() -> m_intake.setIntakePower(-1), m_intake));
    DriverYButton.onFalse(new InstantCommand(() -> m_intake.setIntakePower(0), m_intake));

    DriverLeftBumper.onTrue(new InstantCommand(() -> m_shooter.setShooterPower(-0.25), m_shooter));
    DriverLeftBumper.onFalse(new InstantCommand(() -> m_shooter.setShooterPower(0), m_shooter));
    DriverDPadLeft.onTrue(new InstantCommand(()-> m_robotDrive.toggleYuMode()));

    /*
     * OPERATOR BUTTON MAPPING
     */
  }

//------------------------------------------- autonomous modes -------------------------------------------
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}