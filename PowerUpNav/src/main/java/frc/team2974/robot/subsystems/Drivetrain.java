package frc.team2974.robot.subsystems;

import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;
import static frc.team2974.robot.RobotMap.motorLeft;
import static frc.team2974.robot.RobotMap.motorRight;
import static frc.team2974.robot.RobotMap.pneumaticsShifter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.Config.MotionConstants;
import frc.team2974.robot.Config.Path;
import frc.team2974.robot.Robot;
import frc.team2974.robot.command.teleop.DriveCommand;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.MotionLogger;
import org.waltonrobotics.controller.RobotPair;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.networktables.*;

/**
 *
 */
public class Drivetrain extends AbstractDrivetrain implements PIDOutput {

  private final SendableChooser<Boolean> driveMode;
  public PIDController turnController;
  public double rotateToAngleRate;
  public AHRS ahrs;

  // limelight
  public boolean m_LimelightHasValidTarget = false;
  public double m_LimelightDriveCommand;
  public double m_LimelightSteerCommand;
  public double tv;
  public double tx;
  public double ty; 
  public double ta; 
  public double camtran[];
  private double camTranDefaults[] = {0.0,0.0,0.0,0.0,0.0,0.0};
  
  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system.  Note that the      */
  /* SmartDashboard in Test mode has support for helping you tune    */
  /* controllers by displaying a form where you can enter new P, I,  */
  /* and D constants and test the mechanism.                         */
  static final double kP = 0.015;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;    
  public double kTargetAngleDegrees;
  public boolean isMoving = false;
  public boolean isTurning = false;

  //public Drivetrain() {
  public Drivetrain(MotionLogger motionLogger) {
    super(motionLogger);
    driveMode = new SendableChooser<>();
    driveMode.setDefaultOption("Tank", true);
    //SmartDashboard.putData("Drive Team/Drive Mode", driveMode);

    motorRight.setInverted(true);

    setEncoderDistancePerPulse();

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
      System.out.println("navX initialized on SPI bus.");
    } catch (RuntimeException ex ) {
      //DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-0.75, 0.75);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
    turnController.disable();

    //LiveWindow.addSensor("DriveSystem", "RotateController", turnController);
  }

  @Override
  /* This function is invoked periodically by the PID Controller, */
  /* based upon navX-MXP yaw angle input and PID Coefficients.    */
  public void pidWrite(double output) {
    if (Math.abs(kTargetAngleDegrees - ahrs.getYaw()) <= kToleranceDegrees) {
      rotateToAngleRate = 0;
      //isTurning = false;  // turn is complete
      //turnController.disable();
    } else {
      if ((output > 0.1) && (output < 0.3)) {
        rotateToAngleRate = 0.3;
      } else if ((output < 0.1) && (output > -0.3)) {
        rotateToAngleRate = -0.3;
      } else {
        rotateToAngleRate = output;
      }
    }
  }

  public void SetTargetAngleAbs(float degrees) {
    kTargetAngleDegrees = degrees;
  } 
  
  public void SetTargetAngleRel(float degrees) {
    if ((ahrs.getYaw() + degrees) > 180.0) {
      kTargetAngleDegrees = ahrs.getYaw() + degrees - 360.0; 
    } else if ((ahrs.getYaw() + degrees) < -180.0) {
      kTargetAngleDegrees = ahrs.getYaw() + degrees + 360.0; 
    } else {
      kTargetAngleDegrees = ahrs.getYaw() + degrees;
    }
  }
  
  public void ZeroYaw() {
    ahrs.zeroYaw();
  }
  
  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(encoderLeft.getDistance(), encoderRight.getDistance(),
        Timer.getFPGATimestamp());
  }

  @Override
  public double getRobotWidth() {
    return Robot.getChoosenRobot().getRobotWidth();
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveCommand());
  }

  @Override
  public void reset() {
    System.out.println("Reset Drivetrain");
    encoderLeft.reset();
    encoderRight.reset();
  }

  @Override
  public void setEncoderDistancePerPulse() {
    double distancePerPulse = Robot.getChoosenRobot().getDistancePerPulse();

    encoderLeft.setDistancePerPulse(distancePerPulse);
    encoderRight.setDistancePerPulse(distancePerPulse);
    encoderRight.setReverseDirection(true);
    motorRight.setInverted(true);
  }

  @Override
  public void setSpeeds(double leftPower, double rightPower) {
    //motorRight.set(-rightPower);
    //motorLeft.set(-leftPower);
    motorRight.set(-leftPower);
    motorLeft.set(-rightPower);
  }

  public void shiftDown() {
    pneumaticsShifter.set(true);
  }

  public void shiftUp() {
    pneumaticsShifter.set(false);
  }

  public boolean isShiftDown() {
    return pneumaticsShifter.get();
  }

  public void Update_Limelight_Tracking()
  {
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.02;                    // how hard to turn toward the target
    final double DRIVE_K = 0.10;                    // how hard to drive fwd toward the target
    final double DESIRED_Z_DIST = 35.0;             // Z distance to target
    final double MAX_DRIVE = 0.75;                   // Simple speed limit so we don't drive too fast

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(camTranDefaults);
    
    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
    } else {
      m_LimelightHasValidTarget = true;

      // Start with proportional steering
      double steer_cmd = tx * STEER_K;
      m_LimelightSteerCommand = steer_cmd;

      // try to drive forward until the target area reaches our desired area
      double drive_cmd = 0;
      
      // adjust drive speed
      if (-camtran[2] - DESIRED_Z_DIST > 60) {
        drive_cmd = MAX_DRIVE * 0.85;
      } else if (-camtran[2] - DESIRED_Z_DIST > 40) {
        drive_cmd = MAX_DRIVE * 0.7;   
      } else if (-camtran[2] - DESIRED_Z_DIST > 20) {
        drive_cmd = MAX_DRIVE * 0.5;     
      } else if (ta > 0 && camtran[2] == 0) {
        drive_cmd = MAX_DRIVE;
      } else {
        drive_cmd = 0.3;
      }

      // don't let the robot drive too fast or too slow into the target
      if (drive_cmd > MAX_DRIVE) {
        drive_cmd = MAX_DRIVE;
      } else if (drive_cmd > 0 && drive_cmd < 0.3) {
        drive_cmd = 0.3;
      }

      m_LimelightDriveCommand = -drive_cmd;
    }
  }

  @Override
  public double getKV() {
    return MotionConstants.KV;
  }

  @Override
  public double getKAcc() {
    return MotionConstants.KAcc;
  }

  @Override
  public double getKK() {
    return MotionConstants.KK;
  }

  @Override
  public double getKS() {
    return MotionConstants.KS;
  }

  @Override
  public double getKAng() {
    return MotionConstants.KAng;
  }

  @Override
  public double getKL() {
    return MotionConstants.KL;
  }

  @Override
  public double getILag() {
    return MotionConstants.IL;
  }

  @Override
  public double getIAng() {
    return MotionConstants.IAng;
  }

  @Override
  public double getMaxVelocity() {
    return Path.VELOCITY_MAX;
  }

  @Override
  public double getMaxAcceleration() {
    return Path.ACCELERATION_MAX;
  }

  public boolean isTankDrive() {
    return driveMode.getSelected();
  }

}
