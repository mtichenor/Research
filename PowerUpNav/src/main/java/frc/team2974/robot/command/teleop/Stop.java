package frc.team2974.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.OI;
import frc.team2974.robot.Robot;

/**
 *
 */
public class Stop extends Command {

  public Stop() {
    requires(Robot.drivetrain);
  }

  protected void initialize() {
    System.out.println("Initialize");
    Robot.drivetrain.setSpeeds(0, 0);

  }

  protected void execute() {
    System.out.println("Execute");
    Robot.drivetrain.setSpeeds(0, 0);
  }

  protected boolean isFinished() {
    return false;
  }
}
