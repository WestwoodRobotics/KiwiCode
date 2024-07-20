
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.intake.UTBIntakeCommand;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> m_shooter.setShooterPower(0.85), m_shooter).andThen(new WaitCommand(2)).andThen(new InstantCommand(() -> m_preRoller.setPreRollerPower(1), m_shooter)).andThen(new WaitCommand(1)).andThen(new InstantCommand(() -> m_preRoller.setPreRollerPower(0), m_shooter).alongWith(new InstantCommand(() -> m_shooter.setShooterPower(0), m_shooter))));
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Configure default commands 
    // m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    

    

    // m_chooser.addOption("just shoot", justShootAuto());
    // m_chooser.addOption("shoot pickup from middle shoot", shootPickupShoot());
    // m_chooser.addOption("Three Notes (Preload, Amp & Middle)", centerNoteTopAuto());
    // m_chooser.addOption("Three Notes (Preload, Source & Middle)", centerNoteBottomAuto());
    // m_chooser.addOption("get middle notes out" , messUpNotesAuto());
    // m_chooser.addOption("two note", twoNoteAuto());
    
    // SmartDashboard.putData(m_chooser);

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

    driverLeftTrigger.onTrue(new InstantCommand(() -> m_intake.setIntakePower(1.0), m_intake).alongWith(new InstantCommand(() -> m_preRoller.setPreRollerPower(1.0), m_preRoller)).alongWith(new InstantCommand(() -> m_shooter.setShooterPower(-0.25), m_shooter)));
    driverLeftTrigger.onFalse(new InstantCommand(() -> m_intake.setIntakePower(0.0), m_intake).alongWith(new InstantCommand(() -> m_preRoller.setPreRollerPower(0.0), m_preRoller)));
    driverRightTrigger.onTrue(new InstantCommand(() -> m_shooter.setShooterPower(0.85), m_shooter));
    driverRightTrigger.onFalse(new InstantCommand(() -> m_shooter.setShooterPower(0), m_shooter));

    DriverDPadDown.onTrue(new InstantCommand(() -> m_shooter.setShooterPower(-0.85), m_shooter));
    DriverBButton.onTrue(new InstantCommand(() -> m_preRoller.setPreRollerPower(1), m_preRoller));
    DriverBButton.onFalse(new InstantCommand(() -> m_preRoller.setPreRollerPower(0), m_preRoller));
    DriverXButton.onTrue(new InstantCommand(() -> m_preRoller.setPreRollerPower(-1), m_preRoller));
    DriverXButton.onFalse(new InstantCommand(() -> m_preRoller.setPreRollerPower(0), m_preRoller));

    DriverAButton.onTrue(new InstantCommand(() -> m_intake.setIntakePower(1), m_intake));
    DriverAButton.onFalse(new InstantCommand(() -> m_intake.setIntakePower(0), m_intake));
    DriverYButton.onTrue(new InstantCommand(() -> m_intake.setIntakePower(-1), m_intake));
    DriverYButton.onFalse(new InstantCommand(() -> m_intake.setIntakePower(0), m_intake));

    /*
     * OPERATOR BUTTON MAPPING
     */
  }

//------------------------------------------- autonomous modes -------------------------------------------

    public Command twoNoteAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.35,5.58, new Rotation2d(0)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0))
            // new PathPlannerAuto("TwoNoteAuton")
        );
        toReturn.setName("two note");
        return toReturn;
    }

    // public Command justShootAuto(){
    //    Command toReturn = new PathPlannerAuto("JustShootAuton");
    //    toReturn.setName("just shoot");
    //    return toReturn;
    // }

    public Command shootPickupShoot(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(2*Math.PI/3)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(120))

        );
        toReturn.setName("shoot pickup from middle shoot");
        return toReturn;
    }

    // public Command ampSide(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(-Math.PI/3)))),
    //         new InstantCommand(() -> m_robotDrive.setGyroYawOffset(60)),
    //         new PathPlannerAuto("AmpSideAuton")
    //     );
    // }

    // public Command testingOtherSideSubwoofer(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.74,6.68, new Rotation2d(Math.PI/3)))),
    //         new InstantCommand(() -> m_robotDrive.setGyroYawOffset(60)),
    //         new PathPlannerAuto("TestingOtherSideSubwooferAuton")
    //     );
    // }

	public Command centerNoteTopAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.09,5.56, new Rotation2d(0)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0))
            // new PathPlannerAuto("CenterTopNotesAuton")
           );
        toReturn.setName("Three Notes (Preload, Amp & Middle)");
        return toReturn;
    }

    public Command centerNoteBottomAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.09,5.56, new Rotation2d(0)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0))
            // new PathPlannerAuto("CenterBottomNotesAuton")
        );
        toReturn.setName("Three Notes (Preload, Source & Middle)");
        return toReturn;
    }

    public Command messUpNotesAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.34,5.58, new Rotation2d(0)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0))
            //new PathPlannerAuto("MoveCenterNotesAwayAuton")
        );
        toReturn.setName("get middle notes out");
        return toReturn;
    }

    public Command ExampleAutoInOut(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.34,4.56, new Rotation2d()))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0.0)),
            new PathPlannerAuto("ShootAndOut")
        );
        toReturn.setName("ExampleAutoInOut");
        return toReturn;
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return (this.ExampleAutoInOut());
    // SmartDashboard.putString("selected auto", m_chooser.getSelected().getName());
    // System.out.println(m_chooser.getSelected().getName());
    // if(m_chooser.getSelected().getName().equals("two note")){
    //     System.out.println("^");
    //     return twoNoteAuto();
    // }
    // else if(m_chooser.getSelected().getName().equals("get middle notes out")){
    //     System.out.println("^^");
    //     return messUpNotesAuto();
    // }
    // else if(m_chooser.getSelected().getName().equals("Three Notes (Preload, Amp & Middle)")){
    //     System.out.println("^^^");
    //     return centerNoteTopAuto();
    // }
    // else if(m_chooser.getSelected().getName().equals("shoot pickup from middle shoot")){
    //     System.out.println("^^^^");
    //     return shootPickupShoot();
    // }
    // else if((m_chooser.getSelected().getName().equals("Three Notes (Preload, Source & Middle)"))){
    //     System.out.println("^^^^^");
    //     return centerNoteBottomAuto();
    // }
    // else{
    //     System.out.println("^^^^^^^");
    //     return justShootAuto();
    // }
    //return m_chooser.getSelected();
  }
}