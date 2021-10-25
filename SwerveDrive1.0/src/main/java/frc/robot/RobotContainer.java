// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.auto_commands.FollowTrajectory;
import frc.robot.commands.drive_commands.ChangeSpeed;
import frc.robot.commands.drive_commands.FieldOriented;
import frc.robot.commands.drive_commands.SwerveDrive;
import frc.robot.commands.drive_commands.SwitchDriveMode;
import frc.robot.functional.trajectory.Circle;
import frc.robot.functional.trajectory.Line;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.JoystickRemote;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.XboxRemote;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //subsystems

  public DriveTrain driveTrain = new DriveTrain();
 
  public XboxController xboxController = new XboxController(Constants.xbox_port);
  public XboxController leftJoyStick = new XboxController(Constants.left_joystick_port);
  public XboxController rightJoyStick = new XboxController(Constants.right_joystick_port);
  public JoystickRemote joystick = new JoystickRemote(leftJoyStick, rightJoyStick);
  public XboxRemote xboxRemote = new XboxRemote(xboxController);
  public Odometry odometry = new Odometry();
  public NavXGyro navx = new NavXGyro();
 public LimeLight limeLight = new LimeLight(odometry);
  //buttons


  Button leftPad = new POVButton(xboxController, Constants.left_pad_num);
  Button rightPad = new POVButton(xboxController, Constants.right_pad_num);
  Button upPad = new POVButton(xboxController, Constants.up_pad_num);
  Button downPad = new POVButton(xboxController, Constants.down_pad_num);
  
  Button rightButton = new JoystickButton(xboxController, Constants.rb_button_num);
  Button leftButton = new JoystickButton(xboxController, Constants.lb_button_num);
  Button xButton = new JoystickButton(xboxController, Constants.x_button_num);

  public RobotContainer() {
    // configures commands
    odometry.setDriveTrain(driveTrain);
    FieldOriented sd = new FieldOriented(driveTrain, xboxRemote);
    sd.addRequirements(driveTrain);

    driveTrain.setDefaultCommand(sd);
    
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public Runnable Switch = new Runnable() {
    @Override 
    public void run() {
      SmartDashboard.putBoolean("change D", true);
      if(Constants.drive_mode == 0){
        FieldOriented fo = new FieldOriented(driveTrain, xboxRemote);
        fo.addRequirements(driveTrain);
        driveTrain.setDefaultCommand(fo);
  
        Constants.drive_mode = 1;
      }
      else if(Constants.drive_mode == 1){
        SwerveDrive fo = new SwerveDrive(driveTrain, xboxRemote);
        fo.addRequirements(driveTrain);
        driveTrain.setDefaultCommand(fo);
      }
    }
  };
  private void configureButtonBindings() {
   xButton.whenPressed(new SwitchDriveMode(driveTrain, xboxRemote));
   leftButton.whenPressed(new ChangeSpeed(-0.5));
   rightButton.whenPressed(new ChangeSpeed(0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    double[][] points = {
      {0,0},
      {0,2},
      {1,2},
      {1,3},
      {0,3},
      {0,0}
      };
      double[] distances = {
      0.4,
      0.4,
      0.4,
      0.4
        };
    return new FollowTrajectory(points, distances, driveTrain, odometry);//new AutonomusCommands(driveTrain);
        //return new DriveStraightDistance(1, 1, driveTrain);
        
      //return new AlignAngleRange(driveTrain, limeLight);
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
