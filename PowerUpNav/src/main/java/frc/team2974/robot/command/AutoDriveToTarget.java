package frc.team2974.robot.command;

import static frc.team2974.robot.Robot.drivetrain;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoDriveToTarget extends Command {

  CommandGroup parentCommandGroup;

  public AutoDriveToTarget(CommandGroup callerCommandGroup) {
    requires(drivetrain);
    parentCommandGroup = callerCommandGroup;

    System.out.println("AutoDriveToTarget initialized");
    }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("AutoDriveToTarget started");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double leftPower = 0;
    double rightPower = 0;
  
    if (drivetrain.limelightHasValidTarget) {
      // auto drive to target
      //System.out.println("LimeLight auto drive value: " + drivetrain.limelightDriveCommand);
      leftPower = drivetrain.limelightDriveCommand - drivetrain.limelightSteerCommand;
      rightPower = drivetrain.limelightDriveCommand + drivetrain.limelightSteerCommand;
    } else {
      System.out.println("AutoDriveToTarget cancelled - no target acquired!");
      parentCommandGroup.cancel();  // cancel all remaining commands in command group
    }

    // set speeds
    drivetrain.setSpeeds(leftPower, rightPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    boolean isComplete = false;

    if (drivetrain.limelightDriveCommand == 0) {
      isComplete = true;
    }

    return isComplete;
  }

  // Called once after isFinished returns true
  protected void end() {
    System.out.println("AutoDriveToTarget completed");
    drivetrain.setSpeeds(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
