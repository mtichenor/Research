package frc.team2974.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;
import static frc.team2974.robot.RobotMap.motorLeft;
import static frc.team2974.robot.RobotMap.motorRight;
import static frc.team2974.robot.RobotMap.pneumaticsShifter;
import frc.team2974.robot.Robot;
import frc.team2974.robot.command.teleop.DriveCommand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.*;

public class Drivetrain extends Subsystem {

  public PIDController turnController;
  public PIDController distanceController;
  public double rotateToAngleRate;
  public double driveToDistanceRate;
  public AHRS ahrs;

  // limelight
  public boolean limelightHasValidTarget = false;
  public double limelightDriveCommand;
  public double limelightSteerCommand;
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
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 1.5f;    
  public float kTargetAngleDegrees;

  public Drivetrain() {
    //public Drivetrain(MotionLogger motionLogger) {
    //super(motionLogger);
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

    turnController = new PIDController(kP, kI, kD, kF, ahrs, turnControllerOut, 0.02);
    turnController.setInputRange(-180.0, 180.0);
    turnController.setOutputRange(-0.9, 0.9);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
    turnController.disable();
    //LiveWindow.addSensor("DriveSystem", "RotateController", turnController);

    distanceController = new PIDController(2.0, 0.0, 0.0, 0.0, distanceSource, distanceControllerOut, 0.02);
    distanceController.setInputRange(-180.0, 180.0);
    distanceController.setOutputRange(-0.9, 0.9);
    distanceController.setAbsoluteTolerance(0.05);
    distanceController.setContinuous(false);
    distanceController.disable();

    updateLimelightTracking();
  }

  // Define distance PIDSource
  PIDSource distanceSource = new PIDSource() {
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      // Distance type PID
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return -encoderLeft.getDistance();
    }
  };

  PIDOutput turnControllerOut = new PIDOutput() {
    @Override
    public void pidWrite(double output) {
      if ((output > 0.1) && (output < 0.3)) {
        rotateToAngleRate = 0.3;
      } else if ((output < -0.1) && (output > -0.3)) {
        rotateToAngleRate = -0.3;
      } else {
        rotateToAngleRate = output;
      }
    }
  };

  PIDOutput distanceControllerOut = new PIDOutput() {
    @Override
    public void pidWrite(double output) {
      if ((output > 0.1) && (output < 0.3)) {
        driveToDistanceRate = 0.3;
      } else if ((output < -0.1) && (output > -0.3)) {
        driveToDistanceRate = -0.3;
      } else {
        driveToDistanceRate = output;
      }

      //System.out.println("Distance controller rate: " + driveToDistanceRate);
    }
  };

  public void SetTargetAngleAbs(float degrees) {
    kTargetAngleDegrees = degrees;
  } 
  
  public void SetTargetAngleRel(float degrees) {
    float angle;

    angle = ahrs.getYaw() + degrees;
    if (angle > 180.0) {
      kTargetAngleDegrees = angle - 360.0f; 
    } else if (angle < -180.0) {
      kTargetAngleDegrees = angle + 360.0f; 
    } else {
      kTargetAngleDegrees = angle;
    }
  }
  
  public double Spin(float angle) {
    if (!turnController.isEnabled()) {
      SetTargetAngleRel(angle);
      System.out.println("Spin " + angle + " degrees to: " + kTargetAngleDegrees);  
      SetTurnController(kTargetAngleDegrees);
    }

    return rotateToAngleRate;
  }  
  
  public double RotateTo(float angle) {
    if (!turnController.isEnabled()) {
      SetTargetAngleAbs(angle);
      System.out.println("Rotate to " + kTargetAngleDegrees + " degrees.");
      SetTurnController(kTargetAngleDegrees);
    }

    return rotateToAngleRate;
  }

  public void SetTurnController (double angle) {
    if (!turnController.isEnabled()) {
      turnController.setSetpoint(angle);
      System.out.println("Turn controller heading set to " + turnController.getSetpoint() + " degrees.");
      rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
      turnController.enable();
    }
  }

  public void SetDistanceController (double distance) {
    if (!distanceController.isEnabled()) {
      distanceController.setSetpoint(-encoderLeft.getDistance() + distance);
      System.out.println("Distance controller set to " + distanceController.getSetpoint() + " meters.");
      driveToDistanceRate = 0; // This value will be updated in the pidWrite() method.
      distanceController.enable();
    }
  }

  public void ZeroYaw() {
    ahrs.zeroYaw();
  }

  protected void initDefaultCommand() {
    setDefaultCommand(new DriveCommand());
  }

  public void reset() {
    System.out.println("Reset Drivetrain");
    encoderLeft.reset();
    encoderRight.reset();
  }

  public void setEncoderDistancePerPulse() {
    double distancePerPulse = Robot.getChoosenRobot().getDistancePerPulse();

    encoderLeft.setDistancePerPulse(distancePerPulse);
    encoderRight.setDistancePerPulse(distancePerPulse);
    encoderRight.setReverseDirection(true);
    motorRight.setInverted(true);
  }

  public void setSpeeds(double leftPower, double rightPower) {
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

  public void updateLimelightTracking()
  {
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.015;                    // how hard to turn toward the target
    //final double DRIVE_K = 0.10;                   // how hard to drive fwd toward the target
    final double DESIRED_Z_DIST = 20.0;              // Z distance to target
    final double MAX_DRIVE = 0.80;                   // Simple speed limit so we don't drive too fast

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(camTranDefaults);
    
    if (tv < 1.0) {
      limelightHasValidTarget = false;
      limelightDriveCommand = 0.0;
      limelightSteerCommand = 0.0;
    } else {
      limelightHasValidTarget = true;

      // Start with proportional steering
      double steer_cmd = tx * STEER_K;
      limelightSteerCommand = steer_cmd;

      // try to drive forward until the target area reaches our desired area
      double drive_cmd = 0;
      
      // adjust drive speed
      if (-camtran[2] - DESIRED_Z_DIST > 80) {
        drive_cmd = MAX_DRIVE;
      } else if (-camtran[2] - DESIRED_Z_DIST > 60) {
        drive_cmd = MAX_DRIVE * 0.85;   
      } else if (-camtran[2] - DESIRED_Z_DIST > 40) {
        drive_cmd = MAX_DRIVE * 0.65;   
      } else if (-camtran[2] - DESIRED_Z_DIST > 20) {
        drive_cmd = MAX_DRIVE * 0.5;     
      } else if (ta > 0 && camtran[2] == 0) {
        drive_cmd = MAX_DRIVE;
      } else if (-camtran[2] > DESIRED_Z_DIST) {
        drive_cmd = 0.0;
      } else {
        drive_cmd = 0.3;
      }

      // don't let the robot drive too fast or too slow into the target
      if (drive_cmd > MAX_DRIVE) {
        drive_cmd = MAX_DRIVE;
      } else if (drive_cmd > 0 && drive_cmd < 0.3) {
        drive_cmd = 0.3;
      }

      limelightDriveCommand = -drive_cmd;
    }
  }
}
