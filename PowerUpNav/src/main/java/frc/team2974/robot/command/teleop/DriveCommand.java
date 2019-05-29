package frc.team2974.robot.command.teleop;

import static frc.team2974.robot.OI.gamepad;
import static frc.team2974.robot.Robot.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
//import frc.team2974.robot.OI;

/**
 *
 */
public class DriveCommand extends Command {

  public DriveCommand() {
    requires(drivetrain);
  }

  public double getLeftThrottle() {
    if (Math.abs(gamepad.getLeftY()) < 0.3) {
      return 0;
    }
    return gamepad.getLeftY();
  }

  public double getRightThrottle() {
    if (Math.abs(gamepad.getRightY()) < 0.3) {
      return 0;
    }
    return gamepad.getRightY();
  }

  private void tankDrive() {
    double leftPower = 0;
    double rightPower = 0;
    // check mode
    // use a sequential action queue and check drivetrain.isMoving() to determine when 
    // to move to next action 


    // see what is pressed
    if (gamepad.getButton(4)) {
      //drivetrain.ZeroYaw();  // this will offset the Yaw, so do not use
      leftPower = -drivetrain.RotateTo(4, 0); 
      rightPower = -leftPower; 
    } else if (gamepad.getButton(1)) {
      // spin -90 degrees
      leftPower = -drivetrain.Spin(1, -90); 
      rightPower = -leftPower; 
    } else if (gamepad.getButton(2)) {
      // spin 180 degrees clockwise
      leftPower = -drivetrain.Spin(2, 180); 
      rightPower = -leftPower;  
    } else if (gamepad.getButton(3)) {
      // spin 90 degrees 
      leftPower = -drivetrain.Spin(3, 90); 
      rightPower = -leftPower;       
    } else if (gamepad.getButton(8) && drivetrain.limelightHasValidTarget) {
      // auto drive to target
      if (drivetrain.turnController.isEnabled()) {
        drivetrain.turnController.disable();
      }

      leftPower = drivetrain.limelightDriveCommand - drivetrain.limelightSteerCommand;
      rightPower = drivetrain.limelightDriveCommand + drivetrain.limelightSteerCommand;   
    } else if (gamepad.getButton(7)) {
      // maintain current heading with speed of average of left and right throttles
      drivetrain.SetTurnController(drivetrain.ahrs.getYaw());
      double magnitude = (getLeftThrottle() + getRightThrottle()) / 2;
      leftPower = magnitude - drivetrain.rotateToAngleRate;
      rightPower = magnitude + drivetrain.rotateToAngleRate;
    } else if (gamepad.getButton(5)) {
      // drive forward 1 meter at current heading
      drivetrain.SetTurnController(drivetrain.ahrs.getYaw());
      drivetrain.SetDistanceController(1.0);
      leftPower = -drivetrain.driveToDistanceRate - drivetrain.rotateToAngleRate;;
      rightPower = -drivetrain.driveToDistanceRate + drivetrain.rotateToAngleRate;
  } else {
      // manual tank
      if (drivetrain.turnController.isEnabled()) {
        drivetrain.turnController.disable();
      }     
      if (drivetrain.distanceController.isEnabled()) {
        drivetrain.distanceController.disable();
      }

      leftPower = getLeftThrottle() * 0.75;  // scale back power
      rightPower = getRightThrottle() * 0.75;
    }

    // set speeds
    drivetrain.setSpeeds(leftPower, rightPower);
    
    // are we moving
    if ((leftPower > 0) || (rightPower > 0)) {
      drivetrain.isMoving = true;
    } else {
      drivetrain.isMoving = false;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (drivetrain.isTankDrive()) {
      tankDrive();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
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
