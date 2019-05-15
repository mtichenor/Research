package frc.team2974.robot;

import static frc.team2974.robot.Config.Hardware.LEFT_ENCODER_CHANNEL1;
import static frc.team2974.robot.Config.Hardware.LEFT_ENCODER_CHANNEL2;
import static frc.team2974.robot.Config.Hardware.LEFT_MOTOR_CHANNEL;
import static frc.team2974.robot.Config.Hardware.RIGHT_ENCODER_CHANNEL1;
import static frc.team2974.robot.Config.Hardware.RIGHT_ENCODER_CHANNEL2;
import static frc.team2974.robot.Config.Hardware.RIGHT_MOTOR_CHANNEL;
import static frc.team2974.robot.Config.Hardware.ROBOT_IDENTIFIER_CHANNEL;
import static frc.team2974.robot.Config.Hardware.SHIFTER_CHANNEL;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {

  public static final Talon motorLeft;
  public static final Talon motorRight;
  public static final DigitalInput robotIdentifier; //true for compbot false for practice

  public static final Encoder encoderLeft;
  public static final Encoder encoderRight;

  public static final Compressor compressor;
  public static final Solenoid pneumaticsShifter;

  // public static final DigitalInput limitIntake;
  static {
    motorLeft = new Talon(LEFT_MOTOR_CHANNEL);
    motorRight = new Talon(RIGHT_MOTOR_CHANNEL);

    encoderRight = new Encoder(new DigitalInput(RIGHT_ENCODER_CHANNEL1),
        new DigitalInput(RIGHT_ENCODER_CHANNEL2));
    encoderLeft = new Encoder(new DigitalInput(LEFT_ENCODER_CHANNEL1),
        new DigitalInput(LEFT_ENCODER_CHANNEL2));

    compressor = new Compressor();
    compressor.stop();
    System.out.println("Compressor stopped to save battery.");
    pneumaticsShifter = new Solenoid(SHIFTER_CHANNEL);

    robotIdentifier = new DigitalInput(ROBOT_IDENTIFIER_CHANNEL);
  }

  private RobotMap() {
  }
}
