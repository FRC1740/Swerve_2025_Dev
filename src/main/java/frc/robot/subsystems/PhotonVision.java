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

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Networking.LimelightTable;
import frc.robot.constants.VisionConstants;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera cam;
  PhotonCamera cam2;
  PhotonPoseEstimator Cam2PoseEstimator;
  PhotonPoseEstimator Cam1PoseEstimator;
  PhotonTrackedTarget bestTarget;
  PhotonPipelineResult lastResult;
  String lastCamName;

  public PhotonVision() {
    LimelightTable.getInstance();
    cam = new PhotonCamera(VisionConstants.camName);
    cam2 = new PhotonCamera(VisionConstants.cam2Name);
    cam.setDriverMode(false);
    cam2.setDriverMode(false);

    Cam1PoseEstimator = new PhotonPoseEstimator(
      VisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.RobotToCam1);
      // TODO! not enabled MULTI_TAG_PNP_ON_COPROCESSOR
    Cam2PoseEstimator = new PhotonPoseEstimator(
      VisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.RobotToCam2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("cam.getLatestResult(): " + cam.getLatestResult());
    PhotonPipelineResult result = getLatestResult();
    if (result != null) {
      lastResult = result;
    }
  }

  public PhotonPipelineResult getLatestResult(){
    PhotonPipelineResult result = null;

    List<PhotonPipelineResult> resultList = cam.getAllUnreadResults();
    if (!resultList.isEmpty()) {
      result = resultList.get(resultList.size() - 1);
      if (result.hasTargets()) { 
        bestTarget = result.getBestTarget();
        lastCamName = VisionConstants.camName;
        return result;
      }
    }

    resultList = cam2.getAllUnreadResults();
    if (!resultList.isEmpty()) {
      result = resultList.get(resultList.size() - 1);
      if (result.hasTargets()) { 
        bestTarget = result.getBestTarget();
        lastCamName = VisionConstants.cam2Name;
        return result;
      }
    }
    
    return null;
  }

  public Optional<EstimatedRobotPose> getVisionPoseEstimationResult(){
    if (lastResult != null) {
      if (lastResult.hasTargets()) {
        if (lastCamName == "Cam1") {
          return Cam1PoseEstimator.update(lastResult);
        }else {
          return Cam2PoseEstimator.update(lastResult);
        }
      }
    }

    return null;
  }

  public EstimatedRobotPose ifExistsGetEstimatedRobotPose(){
    Optional<EstimatedRobotPose> estimatedPose = getVisionPoseEstimationResult();

    if (estimatedPose != null) { 
      if (estimatedPose.isPresent()){
        return estimatedPose.get();
      } 
    }
    // System.out.println("null pose");
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