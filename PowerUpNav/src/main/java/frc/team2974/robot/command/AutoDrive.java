/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2974.robot.command;

import edu.wpi.first.wpilibj.command.CommandGroup;
import static frc.team2974.robot.Robot.drivetrain;

public class AutoDrive extends CommandGroup {

  public AutoDrive(int choice) {
    requires(drivetrain);

    System.out.println("AutoDrive choice:  " + choice);
    if (choice == 1) {
      addSequential(new DriveCommandAuto(2, 90));
      addSequential(new DriveCommandAuto(2, 180));
      addSequential(new DriveCommandAuto(2, -90));
      addSequential(new DriveCommandAuto(1, 0));
    }
    else if (choice == 2) {
      addSequential(new DriveCommandAuto(2, 180));
      addSequential(new DriveCommandAuto(1, 0));
      addSequential(new DriveCommandAuto(4, 0.5));  
      addSequential(new DriveCommandAuto(4, 0.5));      
    }

    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
