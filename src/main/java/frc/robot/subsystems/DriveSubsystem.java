// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import frc.Board.CurrentDrawTab;
import frc.Board.DriveTrainTab;
import frc.Board.DriverTab;
import frc.robot.RobotShared;
import frc.robot.constants.CanIds;
import frc.robot.constants.GyroConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.SubsystemConstants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;

public class DriveSubsystem extends SubsystemBase {

  /** gyro angular offset in degrees <b>after</b> auto*/
  double gyroAutoAngularOffset = 0; 

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    CanIds.kFrontLeftDrivingCanId,
    CanIds.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    CanIds.kFrontRightDrivingCanId,
    CanIds.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    CanIds.kRearLeftDrivingCanId,
    CanIds.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    CanIds.kRearRightDrivingCanId,
    CanIds.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

  private RobotShared m_robotShared = RobotShared.getInstance();
  // The gyro sensor
  private AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI); // unsure NavXComType.kMXP_SPI

  
  private final CurrentDrawTab m_CurrentDrawTab = CurrentDrawTab.getInstance();
  private final DriveTrainTab m_DriveTrainTab = DriveTrainTab.getInstance();
  private final DriverTab m_DriverTab = DriverTab.getInstance();
  
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private PhotonVision m_PhotonVision = m_robotShared.getPhotonVision();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    getRotation2d().plus(new Rotation2d(GyroConstants.kGyroAngularOffset)),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    });
//Pose Estimator
  SwerveDrivePoseEstimator PoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    getRotation2d(), 
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    }, new Pose2d());
    
  //Vision
  // PhotonVision m_photonVision;
  
  NetworkTable DriveTrainTable = NetworkTableInstance.getDefault().getTable("DriveTrain");
  
  //Pose data Publisher
  StructArrayPublisher<Pose2d> PosePublisher = DriveTrainTable
  .getStructArrayTopic("Poses", Pose2d.struct).publish();
  
  StructArrayPublisher<Pose2d> Cam1Publisher = DriveTrainTable
    .getStructArrayTopic("Cam1", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> Cam2Publisher = DriveTrainTable
    .getStructArrayTopic("Cam2", Pose2d.struct).publish();

  StructPublisher<Pose2d> OdometryPublisher = DriveTrainTable
    .getStructTopic("Odometry", Pose2d.struct).publish();

  StructArrayPublisher<SwerveModuleState> SwerveModuleStatePublisher = DriveTrainTable
    .getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct).publish();

  StructArrayPublisher<SwerveModuleState> DesiredSwerveModuleStatePublisher = DriveTrainTable
    .getStructArrayTopic("DesiredSwerveModuleStates", SwerveModuleState.struct).publish();


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // configureHolonomic();
  }

  public void configureHolonomic() {
    // AutoBuilder.configureHolonomic(
    //   this::getPose, 
    //   // this::resetOdometry, 
    //   this::resetOdometry,
    //   this::getChassisSpeeds, 
    //   this::chassisSpeedDrive, 
    //   DriveConstants.kPathFollowerConfig, 
    //   () -> {
    //     // I figure if one of these works it's fine, it only runs once so redundancy is fine
    //     if(m_DriveTrainTab.getIsPathFlipped() == 1) { // expicit path flip, if this is not set, it is fine because it gets driverstation
    //       System.out.println("Flipped SB");
    //       return true;
    //     }
    //     var alliance = DriverStation.getAlliance();
    //     if(alliance.isPresent()) {
    //       System.out.println("Flipped DS " + (alliance.get() == DriverStation.Alliance.Red));
    //       return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     System.out.println("No flip");

    //     return m_robotShared.getAlliance() == DriverStation.Alliance.Red;
    //   }, 
    // this);
  }


  @Override
  public void periodic() {

    m_CurrentDrawTab.setDrivingFrontLeftCurrentDraw(m_frontLeft.getDrivingVoltage());
    m_CurrentDrawTab.setDrivingFrontRightCurrentDraw(m_frontRight.getDrivingVoltage());
    m_CurrentDrawTab.setDrivingRearLeftCurrentDraw(m_rearLeft.getDrivingVoltage());
    m_CurrentDrawTab.setDrivingRearRightCurrentDraw(m_rearRight.getDrivingVoltage());
    m_CurrentDrawTab.setTurningFrontLeftCurrentDraw(m_frontLeft.getTurningVoltage());
    m_CurrentDrawTab.setTurningFrontRightCurrentDraw(m_frontRight.getTurningVoltage());
    m_CurrentDrawTab.setTurningRearLeftCurrentDraw(m_rearLeft.getTurningVoltage());
    m_CurrentDrawTab.setTurningRearRightCurrentDraw(m_rearRight.getTurningVoltage());

    m_DriveTrainTab.setModules(m_frontRight.getPosition().angle.getDegrees(), 
      m_frontLeft.getPosition().angle.getDegrees(), 
      m_rearRight.getPosition().angle.getDegrees(), 
      m_rearLeft.getPosition().angle.getDegrees());

    m_DriveTrainTab.setSpeed(m_frontLeft.getSpeed());

    // Update the odometry in the periodic block
    //Adds vision mesurement to pose estimator
    
    updatePoseEstimater(); // add odomentry

    EstimatedRobotPose pose = m_PhotonVision.ifExistsGetEstimatedRobotPose();

    Pose2d camPose = Pose2d.kZero;
    if (pose != null) {
      // camPose = pose.estimatedPose.toPose2d();
      camPose = new Pose2d(pose.estimatedPose.getX(), pose.estimatedPose.getY(), getRotation2d()); // remove rot because vision rot bad

      if (m_PhotonVision.lastCamName == "Cam1") {
        Cam1Publisher.set(new Pose2d[]{ camPose });
      }else {
        Cam2Publisher.set(new Pose2d[]{ camPose });
      }
      // System.out.println("pose: " + camPose);
      
      PoseEstimator.addVisionMeasurement(
        camPose,
        pose.timestampSeconds);
    }
    
    //Just odometry
    m_odometry.update(
      getRotation2d().plus(new Rotation2d(GyroConstants.kGyroAngularOffset)),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      });

    //Pubilsh pose data to network tables
    PosePublisher.set(new Pose2d[]{
      camPose, //Vision Pose
      m_odometry.getPoseMeters(), // odometry
      // new Pose2d(m_odometry.getPoseMeters().getTranslation().rotateBy(new Rotation2d(Units.degreesToRadians(180.0))), m_odometry.getPoseMeters().getRotation()), //Odometry pose
      PoseEstimator.getEstimatedPosition() // combined
    });

    OdometryPublisher.set(getPose());

    //Publish Swerve data to network tables
    SwerveModuleStatePublisher.set(getModuleStates());
    DesiredSwerveModuleStatePublisher.set(getDesiredSwerveModuleStates());
    
    m_DriveTrainTab.setIMU_PitchAngle((double) m_gyro.getPitch());
    m_DriveTrainTab.setIMU_ZAngle((double) m_gyro.getYaw());
  }

  public void updatePoseEstimater(){
    PoseEstimator.update(  
      getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      });

    
    // Optional<EstimatedRobotPose> result = m_photonVision.getVisionPoseEstimationResult();

    // if (result.isPresent()){
    //   EstimatedRobotPose visionPose = result.get();
    //   PoseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
    // }
    m_DriveTrainTab.setRobotPose(PoseEstimator.getEstimatedPosition());
    m_DriverTab.setRobotPose(PoseEstimator.getEstimatedPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return PoseEstimator.getEstimatedPosition();
  }

  /**         
   * Resets the odometry AND POSE ESIMATION to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    PoseEstimator.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
    pose);
    m_odometry.resetPosition(
      getRotation2d().plus(new Rotation2d(GyroConstants.kGyroAngularOffset)),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
    pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean quadraticInput) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    if(quadraticInput){
      xSpeed = herraFCurve(xSpeed, -.7, 18); // want to try -2.2
      ySpeed = herraFCurve(ySpeed, -.7, 18);
      // xSpeed = quadraticControlFalloff(xSpeed);
      // ySpeed = quadraticControlFalloff(ySpeed);
      rot = quadraticControlFalloff(rot);
    }

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed) + Units.degreesToRadians(-gyroAutoAngularOffset);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * m_DriveTrainTab.getMaxDrivingSpeed();
    double ySpeedDelivered = ySpeedCommanded * m_DriveTrainTab.getMaxDrivingSpeed();
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d()) //TODO: add offset
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, m_DriveTrainTab.getMaxDrivingSpeed());
      
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
    //chassis speeds object for use with pathplannerlib
  public void chassisSpeedDrive(ChassisSpeeds speeds){
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_DriveTrainTab.getMaxDrivingSpeed());
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public double quadraticControlFalloff(double input) { // this starts out slow BUT still moves at low values
    return Math.signum(input) * Math.abs((.3 * Math.pow(input, 2))) + (0.7 * input);
  }

  // https://github.com/achilleas-k/fs2open.github.com/blob/joystick_curves/joy_curve_notes/new_curves.md
  /** 
   * HerraFCurve, function taken from Operation Peacce (3461)
   * @param I input value
   * @param s sensitivity of the controls
   * @param degree where the curve starts to be exponential, 9 is middle 4.5 is more linear
  */
  public static double herraFCurve(double I, double s, double degree) {
    double funcInput = Math.abs(I);
    double output = I - (Math.copySign(Math.pow(funcInput, (s / 9)) * Math.pow((1 - Math.cos(funcInput * Math.PI)) / 2, (9 - s) / degree), I)) + I;
    if (Math.abs(I) > .02) {
      // System.out.println("Applied output : " + output);
      return output;
    }else{
      return 0;
    }
  }

  public void setXFormation() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  public void setForwardFormation() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, m_DriveTrainTab.getMaxDrivingSpeed());
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds(){
    SwerveModuleState[] modulestates = getModuleStates();
    return DriveConstants.kDriveKinematics.toChassisSpeeds(modulestates);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }

  public SwerveModuleState[] getDesiredSwerveModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getDesiredState();
    states[1] = m_frontRight.getDesiredState();
    states[2] = m_rearLeft.getDesiredState();
    states[3] = m_rearRight.getDesiredState();
    return states;
  }
  
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    gyroAutoAngularOffset = 0.0;
    System.out.println("Set auto rotation offset to 0.0");
    m_odometry.resetRotation(
      new Rotation2d());
    PoseEstimator.resetRotation(new Rotation2d());
    // m_gyro.setAngleAdjustment(0.0);
  }
  
  // sets the offset after the auto to adjust for starting.
  public void setAutoRotationOffset(double angle, boolean useShuffleboard) {
    if (useShuffleboard) {
        System.out.println("Pulled rotation offset " + m_DriveTrainTab.getAutoRotationOffset());
        // m_gyro.setAngleAdjustment(m_DriveTrainTab.getAutoRotationOffset());
        gyroAutoAngularOffset = m_DriveTrainTab.getAutoRotationOffset();
    } else {
        System.out.println("Set auto rotation offset " + angle);
        gyroAutoAngularOffset = angle;
        // m_gyro.setAngleAdjustment(angle);
    }
}


//Gets gyro in form of Rotation2d
  public Rotation2d getRotation2d(){ // this is the reversed angle and should be used to get the reversed robot angle
    return m_gyro.getRotation2d().plus(new Rotation2d(GyroConstants.kGyroAngularOffset)); // TODO: pose estimator has no rotation offset
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (GyroConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Gets the distance between 2 pose 2ds 
   * @param pose1 the first pose
   * @param pose2 the second pose
  */
  public double getDistance(Pose2d pose1, Pose2d pose2){
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }
}
