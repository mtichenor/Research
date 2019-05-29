package frc.team2974.robot;

//import static frc.team2974.robot.RobotMap.pneumaticsShifter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.subsystems.Drivetrain;
import org.waltonrobotics.MotionLogger;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot { //extends IterativeRobot {

  public static Drivetrain drivetrain;
  public static MotionLogger motionLogger;

  private static Config.Robot currentRobot;


  private int counter = 0;

  //PIDController turnController;
  // TODO:  add rotate, drive to target, drive distance (encoder), combine for auto

  public static Config.Robot getChoosenRobot() {
    return currentRobot;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    currentRobot = RobotMap.robotIdentifier.get() ? Config.Robot.COMPETITION : Config.Robot.PRACTICE;

    motionLogger = new MotionLogger("/home/lvuser/");
    drivetrain = new Drivetrain(motionLogger);

    //		Drive train
    //SmartDashboard.putNumber("Speed %", 0.50 /*.75*/);
    //SmartDashboard.getNumber("Bottom power", .2);

    drivetrain.shiftDown();

    System.out.println("Robot initializing...");
  }

  @Override
  public void disabledInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();
    //motionLogger.writeMotionDataCSV();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
  
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    drivetrain.Update_Limelight_Tracking();
    updateSmartDashboard("Auto");
  }

  @Override
  public void teleopInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.shiftUp(); // start in high gear
    drivetrain.reset();
  }

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    drivetrain.Update_Limelight_Tracking();
    updateSmartDashboard("TeleOp");
  }

  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Put things in here you want to update for SmartDashboard.
   */
  private void updateSmartDashboard(String caller) {
    // only update every 10th call to save CPU and bandwidth (about 500ms)
    if (counter == 50) {
      counter = 0;
    } else {
      counter++;
      return;
    }

    System.out.print(caller + ": " + java.time.LocalTime.now() + " R dist: " + -RobotMap.encoderLeft.getDistance() + " L dist: " + -RobotMap.encoderRight.getDistance());
    System.out.print(" Heading: " +  drivetrain.ahrs.getYaw() + " Rotate rate: " + drivetrain.rotateToAngleRate);
    System.out.println(" 3D LR offset: " + drivetrain.camtran[0] + " 3D dist offset: " + drivetrain.camtran[2]);
    SmartDashboard.putNumber("R_Enc_Dist", -RobotMap.encoderLeft.getDistance());
    SmartDashboard.putNumber("L_Enc_Dist", -RobotMap.encoderRight.getDistance());

    // Drivetrain
    //SmartDashboard.putString("Gear", pneumaticsShifter.get() ? "Low" : "High");

    /* navX */
    /* Display 6-axis Processed Angle Data                                      */
    //SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    //SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    
    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */
    //SmartDashboard.putNumber(   "IMU_C_Head",   ahrs.getCompassHeading());
    
    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    //SmartDashboard.putNumber(   "IMU_F_Head",     ahrs.getFusedHeading());
    // Angle is total degrees including rotations (useful for a turret)
    //SmartDashboard.putNumber(   "IMU_Angle",     ahrs.getAngle());
    SmartDashboard.putNumber(   "IMU_Yaw",     drivetrain.ahrs.getYaw());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    //SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
    //SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

    /* Display estimates of velocity/displacement.  Note that these values are  */
    /* not expected to be accurate enough for estimating robot position on a    */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially      */
    /* double (displacement) integration.                                       */
    //SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
    //SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
    //SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
    //SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
    
    //SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
    
    /* Sensor Board Information                                                 */
    //SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
    
    /* Connectivity Debugging Support                                           */
    //SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
    //SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());

    /* Limelight */
    //SmartDashboard.putNumber(   "Limelight tv",          tv);
    SmartDashboard.putNumber(   "Limelight tx",          drivetrain.tx);
    SmartDashboard.putNumber(   "Limelight ty",          drivetrain.ty);
    SmartDashboard.putNumber(   "Limelight ta",          drivetrain.ta);
    SmartDashboard.putBoolean(   "Limelight target?",            drivetrain.limelightHasValidTarget);
    SmartDashboard.putNumber(   "Limelight 3D x",          drivetrain.camtran[0]);
    //SmartDashboard.putNumber(   "Limelight 3D y",          drivetrain.camtran[1]);
    SmartDashboard.putNumber(   "Limelight 3D z",          drivetrain.camtran[2]);
    
  }
}
  