package frc.team2974.robot;

import static frc.team2974.robot.Config.Input.GAMEPAD_PORT;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team2974.robot.command.teleop.Rotate;
//import frc.team2974.robot.util.ButtonOnce;
//import frc.team2974.robot.util.POVButton;
import static frc.team2974.robot.Robot.drivetrain;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public final class OI {
     
  public static final Gamepad gamepad;

  static {
    gamepad = new Gamepad(GAMEPAD_PORT);
  }
  
  Button button1 = new JoystickButton(gamepad, 1),
  button2 = new JoystickButton(gamepad, 2),
  button3 = new JoystickButton(gamepad, 3), 
  button4 = new JoystickButton(gamepad, 4),
  buttonLeftTrigger = new JoystickButton(gamepad, 7),
  buttonRightTrigger = new JoystickButton(gamepad, 8);

  private OI() {
    /*button1.whenPressed();
    button1.whenPressed(new ZeroYaw(90));
    button2.whenPressed(new Rotate.ZeroYaw());
		button3.whenPressed(new Rotate(-90));*/
  }
}
