// Copyright (c) FIRST and other WPILib contributors.
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.OIConstants;
import frc.robot.constants.SubsystemConstants.HornConstants;
import frc.Board.DriveTrainTab;
import frc.Board.DriverTab;
import frc.Board.GroundIntakeTab;
import frc.Board.HornTab;
import frc.robot.commands.AlignAndDrive.AlignToJoystickAndDrive;
import frc.robot.commands.AlignAndDrive.AlignToNearestAngleAndDrive;
import frc.robot.commands.AlignAndDrive.AlignToTagPhotonvision;
import frc.robot.commands.AlignAndDrive.AlignToTagPose;
import frc.robot.commands.AlignAndDrive.DriveWhileAligning;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private DriveSubsystem m_robotDrive;
  
  private RobotShared m_robotShared = RobotShared.getInstance();

  // PathPlannerPath m_ExamplePath = PathPlannerPath.fromPathFile("Example Path");

  // The driver's controller
  CommandXboxController m_driverController;
  CommandXboxController m_coDriverController;


  SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    initSubsystems();
    initInputDevices();

    // Configure the button bindings
    configureButtonBindings();
    buttonBoardControls();
    // flightStickControls();

    // Configure default commands
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      // If any changes are made to this, please update DPad driver controls
    if(OIConstants.kUseFieldRelativeRotation){
      m_robotDrive.setDefaultCommand(new RunCommand(() -> 
        new AlignToJoystickAndDrive(
          m_driverController.getRightX(),
          m_driverController.getRightY(),
          true, true, 
          (-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) + 
          -MathUtil.applyDeadband(m_driverController.getRightY(), OIConstants.kDriveDeadband) != 0) ? 1 : 0).execute(), m_robotDrive));
    }else{
      m_robotDrive.setDefaultCommand(
        new RunCommand(() -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true, true, OIConstants.kUseQuadraticInput),
          m_robotDrive));
    }
  }

  private void initSubsystems() {
    m_robotShared = RobotShared.getInstance();

    m_robotDrive = m_robotShared.getDriveSubsystem();
    m_robotShared.getPhotonVision();

    DriverTab.getInstance();
  }
  private void initInputDevices() {
    m_coDriverController = m_robotShared.getCoDriverController();
    m_driverController = m_robotShared.getDriverController();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.start()
      .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    m_driverController.y()
      .whileTrue(
        new AlignToTagPose()
      );
    m_driverController.leftTrigger()
      .whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) / 2,
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) / 2,
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) / 2,
          true, true, OIConstants.kUseQuadraticInput),
        m_robotDrive));

    // This is a stick click
    m_driverController.rightStick()
      .onTrue(new SequentialCommandGroup(
        // double normalizedAngle = (int)((m_robotDrive.getHeading() + 180) / (360 / 8)),  // This is the uncondensed code
        new AlignToNearestAngleAndDrive(true, true).withTimeout(3))
      );
    // Something super janky is happening here but it works so
    for(int angleForDPad = 0; angleForDPad <= 7; angleForDPad++) { // Sets all the DPad to rotate to an angle
      new POVButton(m_driverController.getHID(), angleForDPad * 45)
        .onTrue(
          new DriveWhileAligning(angleForDPad * -45, true, true).withTimeout(3)); // -45 could be 45 
    }
    m_driverController.leftStick()
      .whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) / 2,
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) / 2,
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) / 2,
          true, true, OIConstants.kUseQuadraticInput),
        m_robotDrive));
  }

  void buttonBoardControls(){
    // define buttons 
    Trigger buttonBoardButtons[][] = new Trigger[3][3];
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
        buttonBoardButtons[i][j] = m_coDriverController.button((i * 3) + j + 1);
      }
    }

    Trigger buttonBoardSwitches[][] = new Trigger[2][2];
    for(int i = 0; i < 2; i++){
      for(int j = 0; j < 2; j++){
        buttonBoardSwitches[i][j] = m_coDriverController.button((((i * 2) + j) * 2) + 10); // starts st 10 offset
      }
    }
    buttonBoardSwitches[0][1] // top
    .whileTrue(
      new RunCommand(() -> HornTab.getInstance().setHornTargetSpeed(HornConstants.kHornSpeakerShotMotorRPM))
    )
    .onFalse(
      new InstantCommand(() -> HornTab.getInstance().setHornTargetSpeed(0.0))
    );

    buttonBoardSwitches[1][1] // bottom right
    .onTrue(
      new InstantCommand(() -> GroundIntakeTab.getInstance().setGroundIntakeDefaultEnabled(true))
    )
    .onFalse(
      new InstantCommand(() -> GroundIntakeTab.getInstance().setGroundIntakeDefaultEnabled(false))
    );
    // buttonBoardSwitches[1][1] // bottom right
    // .onTrue(
    //   new InstantCommand(() -> m_deflectorSubsystem.resetDeflectorEncoder())
    // );
    
    buttonBoardSwitches[0][0]
    .onTrue(
      new InstantCommand(() -> m_robotDrive.setAutoRotationOffset(0.0, true))
    );
    
  }

  void flightStickControls(){
    m_driverController.x()
      .onTrue(
        new InstantCommand(() -> m_robotDrive.zeroHeading())
      );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
