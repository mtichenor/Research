package frc.team2974.robot;

import static frc.team2974.robot.Config.Input.GAMEPAD_PORT;

//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import frc.team2974.robot.util.ButtonOnce;
//import frc.team2974.robot.util.POVButton;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public final class OI {

  public static final Gamepad gamepad;

  static {
    gamepad = new Gamepad(GAMEPAD_PORT);
  }

  private OI() {
  }
}
