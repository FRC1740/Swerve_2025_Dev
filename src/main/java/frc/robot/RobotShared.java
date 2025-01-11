//RobotShared is the class that holds all the instances of subsystems and other useful systems.
package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotShared {

  private Optional<Alliance> m_alliance = DriverStation.getAlliance();

  protected DriveSubsystem m_robotDrive = null;
  
  protected final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  protected final CommandXboxController m_coDriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);


  protected LimelightSubsystem m_limelight = null;

  private static RobotShared instance;

  public static RobotShared getInstance() {
    if(instance == null) {
      instance = new RobotShared();
    }
    return instance;
  }

  public DriveSubsystem getDriveSubsystem() {
    if(m_robotDrive == null) {
      m_robotDrive = new DriveSubsystem();
    }
    return m_robotDrive;
  }
  public CommandXboxController getDriverController() {
    return m_driverController;
  }
  public CommandXboxController getCoDriverController() {
    return m_coDriverController;
  }
  public LimelightSubsystem getLimelight() {
    if(m_limelight == null) {
      m_limelight = new LimelightSubsystem();
    }
    return m_limelight;
  }
  /** blue is default */
  public Alliance getAlliance() { // blue is default for the path planner (paths are made on the blue side)
    if(DriverStation.isFMSAttached()){
      m_alliance = DriverStation.getAlliance();
      if(m_alliance.isPresent()){
        if(m_alliance.get() == Alliance.Blue){
          return Alliance.Blue;
        }else{
          return Alliance.Red;
        }
      }else{
        System.err.println("No alliance found!");
        return Alliance.Blue;
      }
    }else{
      System.err.println("No FMS found!");
      return Alliance.Blue;
    }
  }
}
