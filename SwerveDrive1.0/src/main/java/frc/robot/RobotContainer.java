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
import frc.robot.functional.Circle;
import frc.robot.functional.Line;
import frc.robot.functional.Wheel;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.SwerveDrive;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private XboxController xc = new XboxController(Constants.xbox_p);
  private Wheel bl = new Wheel(Constants.bl_angle, Constants.bl_speed);
  private Wheel br = new Wheel(Constants.br_angle, Constants.br_speed);
  private Wheel fr = new Wheel(Constants.fr_angle, Constants.fr_speed);
  private Wheel fl = new Wheel(Constants.fl_angle, Constants.fl_speed);

  public SwerveDrive sd = new SwerveDrive(br, bl, fr, fl);
  public TeleDrive td = new TeleDrive(xc, sd);
  public AutoDrive ad = new AutoDrive(sd);
  
  //subsystems
  public Odometry odometry = new Odometry();
  public LimeLight limeLight = new LimeLight();
  Button leftPad;
  Button rightPad;
  Button upPad;
  Button downPad;
  Object[] motors = {sd,td,ad};

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

  public Object[] returnmotors() {return motors;}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public Runnable Switch = new Runnable() {@Override public void run() {if (td.fo == false) {td.fo = true;} else {td.fo = false;}td.checkFO();}};
  private void configureButtonBindings() {
    leftPad.whenPressed(Switch);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public static double navxTo360(double angle){
        
    if (angle<=0) angle += 360;

    return 360-angle;
  }
  public static double to360(double angle) {
    if (angle <= 0) angle += 360;

    return Math.abs(angle%360);
  }
  public static double stickTo360(double x, double y){
   return (Math.toDegrees(Math.atan2(-x, y)) +360 )%360;
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

    if(targetAngle <= 180) value = angle<targetAngle || angle> 180+targetAngle;
    else value = angle<targetAngle && angle> targetAngle-180;
    return value;
  }
 

  public static double distance(double[] p1, double[] p2){
    return Math.sqrt( Math.pow(p1[1] - p2[1], 2) + Math.pow(p1[0] - p2[0], 2));
  }


  public static double angleFromSlope(double[] start, double[] end){
    return Math.toDegrees(Math.atan2((end[1] - start[1]), end[0] - start[0]));
  }
  public static double magnitutde(double[] vector){
    return Math.sqrt((vector[0]*vector[0]) + (vector[1]*vector[1]));
  }

  public static double angleDistance2(double targetAngle, double angle){
    
    double distance = Math.abs(targetAngle - angle)%360;
    if (distance > 180) distance = 360 - distance;
    return distance;
  }


  public static double floorMod(double x, double y){
    if(x<0)
        return y - Math.abs(x)%y;
    return x%y;
}

public static double angleToPoint(double[] start, double[] end){
  double dx = end[0]-start[0];
  double dy = end[1]-start[1];
  return to360(Math.toDegrees(Math.atan2(-dx, dy)));

}
public static double getArcLength(Circle circle){
  Line base = new Line(circle.startPoint, circle.endPoint);
  double[] midPoint = base.getMidPoint();
  double halfAngle = Math.atan(distance(midPoint, base.startPoint)/distance(midPoint, circle.center));
  return halfAngle*2*circle.radius;
}

}
