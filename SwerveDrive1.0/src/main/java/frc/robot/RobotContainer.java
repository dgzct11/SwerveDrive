// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drive_commands.AutoDrive;
import frc.robot.commands.drive_commands.TeleDrive;
import frc.robot.functional.Wheel;
import frc.robot.subsystems.SwerveDrive;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private XboxController xc = new XboxController(Constants.xbox_p);
  private Wheel br = new Wheel(Constants.br_angle, Constants.br_speed);
  private Wheel bl = new Wheel(Constants.bl_angle, Constants.bl_speed);
  private Wheel fr = new Wheel(Constants.fr_angle, Constants.fr_speed);
  private Wheel fl = new Wheel(Constants.fl_angle, Constants.fl_speed);

  public SwerveDrive sd = new SwerveDrive(br, bl, fr, fl);
  public TeleDrive td = new TeleDrive(xc, sd);
  public AutoDrive ad = new AutoDrive(sd);
  
  //subsystems\

  //Buttons
  Button leftPad;
  Button rightPad;
  Button upPad;
  Button downPad;

  //buttons
  public RobotContainer() {
    // configures commands
    //odometry.setDriveTrain(driveTrain);
    leftPad = new POVButton(xc, Constants.left_pad_num);
    rightPad = new POVButton(xc, Constants.right_pad_num);
    upPad = new POVButton(xc, Constants.up_pad_num);
    downPad = new POVButton(xc, Constants.down_pad_num);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public Runnable foSwitch = new Runnable() {public void run() {if (td.fo == false) {td.fo = true;} else {td.fo = false;}td.checkFO();}};
  public Runnable raise = new Runnable() {public void run() {Constants.velocityMax++; Constants.max_motor_percent++;}};
  public Runnable lower = new Runnable() {public void run() {if (Constants.velocityMax != 0||Constants.max_motor_percent != 0) {Constants.velocityMax-=0.5; Constants.max_motor_percent-=0.5;}}};
  private void configureButtonBindings() {
    leftPad.whenPressed(raise);
    rightPad.whenPressed(lower);
    upPad.whenPressed(foSwitch);
  }
}
