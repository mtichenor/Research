package frc.team2974.robot;

import static frc.team2974.robot.RobotMap.pneumaticsShifter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.subsystems.Drivetrain;
import org.waltonrobotics.MotionLogger;
import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot { //extends IterativeRobot {

  public static Drivetrain drivetrain;
  public static MotionLogger motionLogger;

  private static Config.Robot currentRobot;

  private AHRS ahrs;

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand;
  private double m_LimelightSteerCommand;
  private double tv;
  private double tx;
  private double ty; 
  private double ta; 
  private double camtran[];
  private double camTranDefaults[] = {0.0,0.0,0.0,0.0,0.0,0.0};
  private int counter = 0;

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
    SmartDashboard.putNumber("Speed Percentage", 0.50 /*.75*/);
    SmartDashboard.getNumber("Bottom power", .2);

    drivetrain.shiftDown();

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
      System.out.println("navX initialized on SPI bus.");
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    System.out.println("Robot initializing...");
  }

  @Override
  public void disabledInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();
    motionLogger.writeMotionDataCSV();
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
    Update_Limelight_Tracking();
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
    // only update every 10th call to save CPU and bandwidth (about 200ms)
    if (counter == 10) {
      counter = 0;
    } else {
      counter++;
      return;
    }

    System.out.print(caller + ": " + java.time.LocalTime.now() + " R dist: " + -RobotMap.encoderLeft.getDistance() + " L dist: " + -RobotMap.encoderRight.getDistance());
    System.out.print(" Heading: " +  ahrs.getFusedHeading());
    System.out.println(" 3D x: " + camtran[0] + " 3D y: " + camtran[1] + " 3D z: " + camtran[2]);
    //SmartDashboard.putNumber("Left_Enc_Dist", RobotMap.encoderLeft.getDistance());
    //SmartDashboard.putNumber("Right_Enc_Dist", RobotMap.encoderRight.getDistance());
    SmartDashboard.putNumber("Right_Enc_Dist", -RobotMap.encoderLeft.getDistance());
    SmartDashboard.putNumber("Left_Enc_Dist", -RobotMap.encoderRight.getDistance());

    // Drivetrain
    //SmartDashboard.putString("Gear", pneumaticsShifter.get() ? "Low" : "High");

    /* navX */
    /* Display 6-axis Processed Angle Data                                      */
    //SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    //SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    
    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */
    SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
    
    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

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
    SmartDashboard.putNumber(   "Limelight tv",          tv);
    SmartDashboard.putNumber(   "Limelight tx",          tx);
    SmartDashboard.putNumber(   "Limelight ty",          ty);
    SmartDashboard.putNumber(   "Limelight ta",          ta);
    SmartDashboard.putBoolean(   "Limelight valid target",            m_LimelightHasValidTarget);
    SmartDashboard.putNumber(   "Limelight 3D x",          camtran[0]);
    SmartDashboard.putNumber(   "Limelight 3D y",          camtran[1]);
    SmartDashboard.putNumber(   "Limelight 3D z",          camtran[2]);
    
  }

  public void Update_Limelight_Tracking()
  {
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(camTranDefaults);
    
    if (tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }
}
