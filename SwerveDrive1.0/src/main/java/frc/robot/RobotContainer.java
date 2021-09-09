// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.functional.Circle;
import frc.robot.functional.Line;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public static double navxTo360(double angle){
        
    if (angle<=0) angle += 360;

    return 360-angle;
  }
  public static double to360(double angle) {
    if (angle <= 0) angle += 360;

    return Math.abs(angle%360);
  }
  public static double stickTo360(double x, double y){
   return (to360(Math.toDegrees(Math.atan2(-y,x)))+270)%360;
  }
  public static boolean shouldTurnLeftNavx(double currentNavxAngle, double targetAngle){
    double angle = navxTo360(currentNavxAngle);
    boolean value = false;

    if(targetAngle < 180) value = angle<targetAngle || angle> 180+targetAngle;
    else value = angle<targetAngle && angle> targetAngle-180;
    return value;
  }
  public static boolean shouldTurnLeft(double currentNavxAngle, double targetAngle){
    double angle = currentNavxAngle;
    boolean value = false;

    if(targetAngle < 180) value = angle<targetAngle || angle> 180+targetAngle;
    else value = angle<targetAngle && angle> targetAngle-180;
    return value;
  }
 

  public static double distance(double[] p1, double[] p2){
    return Math.sqrt( Math.pow(p1[1] - p2[1], 2) + Math.pow(p1[0] - p2[0], 2));
  }

  public static double getArcLength(Circle circle){
    Line base = new Line(circle.startPoint, circle.endPoint);
    double[] midPoint = base.getMidPoint();
    double halfAngle = Math.atan(distance(midPoint, base.startPoint)/distance(midPoint, circle.center));
    return halfAngle*2*circle.radius;
  }

  public static double angleFromSlope(double[] start, double[] end){
    return Math.toDegrees(Math.atan2((end[1] - start[1]), end[0] - start[0]));
  }
}
