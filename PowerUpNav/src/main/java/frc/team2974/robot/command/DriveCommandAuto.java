package frc.team2974.robot.command;

import static frc.team2974.robot.Robot.drivetrain;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCommandAuto extends Command {

  int autoMode;
  double autoValue;

  public DriveCommandAuto(int mode, double value) {
    requires(drivetrain);
    autoMode = mode;
    autoValue = value;

    System.out.println("DriveCommandAuto initialized: " + autoMode + " value=" + autoValue);
    }

  private void autoDrive() {
    double leftPower = 0;
    double rightPower = 0;
    //System.out.println("autoMode=" + autoMode + " value=" + autoValue);
    // see what is pressed
    if (autoMode == 1) {
      // rotate to absolute angle
      leftPower = -drivetrain.RotateTo((float) autoValue); 
      rightPower = -leftPower; 
    } else if (autoMode == 2) {
      // spin relative angle
      leftPower = -drivetrain.Spin((float) autoValue); 
      rightPower = -leftPower;      
    } else if (autoMode == 3 && drivetrain.limelightHasValidTarget) {
      // auto drive to target
      if (drivetrain.turnController.isEnabled()) {
        drivetrain.turnController.disable();
      }

      leftPower = drivetrain.limelightDriveCommand - drivetrain.limelightSteerCommand;
      rightPower = drivetrain.limelightDriveCommand + drivetrain.limelightSteerCommand;
    } else if (autoMode == 4) {
      // drive forward 1 meter at current heading
      drivetrain.SetTurnController(drivetrain.ahrs.getYaw());
      drivetrain.SetDistanceController(autoValue);
      leftPower = -drivetrain.driveToDistanceRate - drivetrain.rotateToAngleRate;;
      rightPower = -drivetrain.driveToDistanceRate + drivetrain.rotateToAngleRate;
    } else {
      // invalid
      leftPower = 0.0;  
      rightPower = 0.0;
    }

    // set speeds
    drivetrain.setSpeeds(leftPower, rightPower);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("DriveCommandAuto started: " + autoMode + " value=" + autoValue);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    autoDrive();
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    boolean isComplete = false;
    
    switch (autoMode) {
    case 1: case 2:
      if (drivetrain.turnController.onTarget()) {
        if (drivetrain.turnController.isEnabled()) {
          drivetrain.turnController.reset();
        }  
        isComplete = true;
        System.out.println("DriveCommandAuto completed: " + autoMode + " value=" + autoValue);
        break;
      }
      case 3:
      if (drivetrain.distanceController.onTarget()) {
        if (drivetrain.distanceController.isEnabled()) {
          drivetrain.distanceController.disable();
        }
        isComplete = true;
        System.out.println("DriveCommandAuto completed: " + autoMode + " value=" + autoValue);
        }
        break;
      case 4:
      if (drivetrain.turnController.onTarget() && drivetrain.distanceController.onTarget()) {
        if (drivetrain.turnController.isEnabled()) {
          drivetrain.turnController.disable();
        }  
        if (drivetrain.distanceController.isEnabled()) {
          drivetrain.distanceController.disable();
        }
        isComplete = true;
        System.out.println("DriveCommandAuto completed: " + autoMode + " value=" + autoValue);
        }
        break;
      default:
        System.out.println("Invalid auto mode: " + autoMode);
        break;
      }
    return isComplete;
  }

  // Called once after isFinished returns true
  protected void end() {
    drivetrain.setSpeeds(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
