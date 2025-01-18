// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Networking.LimelightTable;
import frc.robot.constants.VisionConstants;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera cam;
  PhotonCamera cam2;
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator PoseEstimator;
  PhotonTrackedTarget bestTarget;
  String lastCamName;

  public PhotonVision() {
    LimelightTable.getInstance();
    cam = new PhotonCamera(VisionConstants.camName);
    cam2 = new PhotonCamera(VisionConstants.cam2Name);
    cam.setDriverMode(false);
    cam2.setDriverMode(false);
    try{
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); }
    catch(IOException IOE){
      IOE.printStackTrace();
    }

    PoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.RobotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("cam.getLatestResult(): " + cam.getLatestResult());
    getLatestResult();
  }

  public PhotonPipelineResult getLatestResult(){
    PhotonPipelineResult result = cam.getLatestResult();

    if (result.hasTargets()) {    
      bestTarget = result.getBestTarget();
      lastCamName = VisionConstants.camName;
      return result;
    }else {
      lastCamName = VisionConstants.cam2Name;
      result = cam2.getLatestResult();
      bestTarget = result.getBestTarget();
      return cam2.getLatestResult();
    }
  }

  public Optional<EstimatedRobotPose> getVisionPoseEstimationResult(){
    return PoseEstimator.update(getLatestResult());
  }

  public EstimatedRobotPose ifExistsGetEstimatedRobotPose(){
    Optional<EstimatedRobotPose> estimatedPose = getVisionPoseEstimationResult();
    if (estimatedPose.isPresent()){
      return estimatedPose.get();
    } 
    return null;
  }

  public PhotonTrackedTarget getBestTarget(){
    return bestTarget;
  }
  public double getBestTargetX(){
    return bestTarget.bestCameraToTarget.getX() - VisionConstants.cameraOffsets.get(lastCamName).getX();
  }
  public double getBestTargetY(){
    return bestTarget.bestCameraToTarget.getY() - VisionConstants.cameraOffsets.get(lastCamName).getY();
  }

  public Transform3d getCamToTarget(){
    return bestTarget.getBestCameraToTarget();
  }

  //Returns list of IDs currently being tracked
  public List<Integer> getAprilTagIDs(){
    List<PhotonTrackedTarget> targets = getLatestResult().getTargets();
    List<Integer> tagIDs = new ArrayList<>();
    targets.forEach(target -> tagIDs.add(target.getFiducialId()));

    return tagIDs;
  }

  //Returns true if an the ID is being tracked
  public boolean containsID(Integer ID){
    return getAprilTagIDs().contains(ID);
  }
}