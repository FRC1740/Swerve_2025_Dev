// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlignAndDrive;

import java.io.Console;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveCommandConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.Robot;
import frc.robot.RobotShared;

public class AlignToTagPose extends Command {
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
  Pose2d targetPose = null;

  NetworkTable DriveTrainTable = NetworkTableInstance.getDefault().getTable("DriveTrain");
  
  //Pose data Publisher
  StructArrayPublisher<Pose2d> PosePublisher = DriveTrainTable
  .getStructArrayTopic("Target Pose", Pose2d.struct).publish();

  public static double normalizeAngle(double angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }   

  /** Creates a new Command that aligns the robot angle to an apriltag using the Limelight. 
   * <br></br> This command <b>DOES DRIVE</b>
  */
  public AlignToTagPose() {
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
      Optional<Pose3d> tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
      // TODO! check if reef tag
      if (tagPose.isPresent()) { 
        targetPose = tagPose.get().toPose2d();
      }
    }
      
    if (targetPose != null) {
      distanceToTag = target.bestCameraToTarget; // getTranslationToAprilTag may be incorrect

      Pose2d rotatedGoal = new Pose2d(DriveCommandConstants.xGoal, DriveCommandConstants.yGoal, new Rotation2d());
      rotatedGoal = rotatedGoal.rotateBy(targetPose.getRotation()); // Rotate the goal to account for rotated tags

      rotatedGoal = new Pose2d(
        (targetPose.getX() + rotatedGoal.getX()), // apply target offsets
        (targetPose.getY() + rotatedGoal.getY()), 
        targetPose.getRotation().plus(new Rotation2d(1 * Math.PI))); // normal of the tag is flipped from robot target
      PosePublisher.set(new Pose2d[] { rotatedGoal });
      
      angleToTag = normalizeAngle(rotatedGoal.getRotation().getRadians() - m_drive.getRotation2d().getRadians());

      x_error = (m_drive.getPose().getX() - rotatedGoal.getX()); // f - b error
      y_error = (m_drive.getPose().getY() - rotatedGoal.getY()); // l - r error
      // flip because mechs on "back"
      theta_error = normalizeAngle(angleToTag);
      System.out.println("angle: " + angleToTag);
      System.out.println("x: " + x_error);
      System.out.println("y: " + y_error);
      // x_error = 0;
      
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
