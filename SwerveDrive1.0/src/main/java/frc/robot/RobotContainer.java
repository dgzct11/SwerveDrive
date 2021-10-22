// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveManager;
import frc.robot.functional.Circle;
import frc.robot.functional.Line;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Odometry;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //subsystems
  
  public XboxController xc = new XboxController(Constants.xbox_p);
  public Odometry odometry = new Odometry();
  public NavXGyro navx = new NavXGyro();
  public LimeLight limeLight = new LimeLight();
  Button leftPad;
  Button rightPad;
  Button upPad;
  Button downPad;
  DriveManager dm = new DriveManager(xc);

  //buttons
  public RobotContainer() {
    // configures commands
    //odometry.setDriveTrain(driveTrain);
    //driveTrain.setDefaultCommand(DriveManager);
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
  private void configureButtonBindings() {
   leftPad.whenPressed(new Runnable(){@Override public void run() {if (dm.drivemode == 0) {dm.drivemode = 1;} else {dm.drivemode = 0;}}});
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public DriveManager getCommand() {return dm;}

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
