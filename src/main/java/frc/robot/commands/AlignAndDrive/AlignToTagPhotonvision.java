// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlignAndDrive;

import java.io.Console;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
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
    PhotonTrackedTarget target = m_photonvision.getBestTarget();
    if (target != null) {
    angleToTag = target.bestCameraToTarget.getRotation().getAngle();
    distanceToTag = target.bestCameraToTarget; // getTranslationToAprilTag may be incorrect

    x_error = -(DriveCommandConstants.xGoal - m_photonvision.getBestTargetX()); // forward back
    y_error = -(DriveCommandConstants.yGoal - m_photonvision.getBestTargetY()); // l - r error
    System.out.println("angle: " + angleToTag);
    System.out.println("x: " + x_error);
    System.out.println("y: " + y_error);
    theta_error = -(angleToTag - (Math.PI));
    // if theta not aligned, don't drive yet
    if(Math.abs(theta_error) > DriveCommandConstants.kThetaToleranceRadians){
      x_error = 0.0;
      y_error = 0.0;
    }


    
    m_drive.drive(
      DriveCommandConstants.kXP * x_error, 
      DriveCommandConstants.kYP * y_error, 
      // 0.0,0.0, // don't drive
      DriveCommandConstants.kThetaP * theta_error, false, true, false);

    if(Math.abs(x_error) < DriveCommandConstants.kXToleranceMeters){
      XFinished = true;
    }
    if(Math.abs(y_error) < DriveCommandConstants.kYToleranceMeters){
      YFinished = true;
    }
    if(Math.abs(theta_error) < DriveCommandConstants.kThetaToleranceRadians){
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
