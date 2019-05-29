/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2974.robot.command.teleop;

import edu.wpi.first.wpilibj.command.InstantCommand;
import static frc.team2974.robot.Robot.drivetrain;


/**
 * Add your docs here.
 */
public class Rotate extends InstantCommand {
  /**
   * Add your docs here.
   */
  public Rotate(double degrees) {
    super();

    System.out.println("Rotate " + degrees + " degrees.");
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(drivetrain);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
   
  }

}
