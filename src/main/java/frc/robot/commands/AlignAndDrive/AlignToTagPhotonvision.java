// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlignAndDrive;

import java.io.Console;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveCommandConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.RobotShared;

public class AlignToTagPhotonvision extends Command {
  private DriveSubsystem m_drive;
  private PhotonVision m_photonvision;

  private RobotShared m_robotShared;

  double x_error;
  double y_error;
  double theta_error;
  double angleToTag;
  Transform3d distanceToTag;
  boolean XFinished;
  boolean YFinished;
  boolean ThetaFinished;

  /** Creates a new Command that aligns the robot angle to an apriltag using the Limelight. 
   * <br></br> This command <b>DOES DRIVE</b>
  */
  public AlignToTagPhotonvision() {
    m_robotShared = RobotShared.getInstance();
    m_drive = m_robotShared.getDriveSubsystem();
    m_photonvision = m_robotShared.getPhotonVision();
    addRequirements(m_drive);
    addRequirements(m_photonvision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonvision.getBestTarget() != null) {
    angleToTag = m_photonvision.getBestTarget().bestCameraToTarget.getX();
    System.out.println("angle: " + angleToTag);
    distanceToTag = m_photonvision.getBestTarget().bestCameraToTarget; // getTranslationToAprilTag may be incorrect

    x_error = DriveCommandConstants.xGoal - distanceToTag.getX();
    y_error = DriveCommandConstants.yGoal - distanceToTag.getY();
    theta_error = angleToTag;


    
    m_drive.drive(
      // DriveCommandConstants.kXP * x_error, 
      // DriveCommandConstants.kYP * y_error, 
      0.0,0.0, // don't drive
      DriveCommandConstants.kThetaP * theta_error, false, true, false);

    if(Math.abs(x_error) < DriveCommandConstants.kXToleranceMeters){
      XFinished = true;
    }
    if(Math.abs(y_error) < DriveCommandConstants.kYToleranceMeters){
      YFinished = true;
    }
    if(Math.abs(theta_error) < DriveCommandConstants.kXToleranceMeters){
      ThetaFinished = true;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XFinished && YFinished && ThetaFinished;
  }
}
