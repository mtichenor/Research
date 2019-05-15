package frc.team2974.robot.command.teleop;

import static frc.team2974.robot.Config.Driving.CRUISE_POWER;
import static frc.team2974.robot.OI.gamepad;
import static frc.team2974.robot.Robot.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.OI;

/**
 *
 */
public class DriveCommand extends Command {

  private boolean cruiseMode;

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
    double leftPower = getLeftThrottle() * 0.7;
    double rightPower = getRightThrottle() * 0.7;

    if (cruiseMode) {
      double powerNormalizer = 1 - Math.abs(CRUISE_POWER);

      leftPower = CRUISE_POWER + (leftPower / powerNormalizer);
      rightPower = CRUISE_POWER + (rightPower / powerNormalizer);
    }

    drivetrain.setSpeeds(leftPower, rightPower);
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
