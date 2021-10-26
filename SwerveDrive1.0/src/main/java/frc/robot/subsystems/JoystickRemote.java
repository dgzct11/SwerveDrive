// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class JoystickRemote extends XboxRemote {
  /** Creates a new XboxRemote. */
  XboxController leftController;
  XboxController rightController;
  public JoystickRemote(XboxController lc, XboxController rc) {
      super(lc);
    leftController = lc;
    rightController = rc;
  }

  public double getLeftAngle(){
    double y = -leftController.getRawAxis(Constants.left_x_axis);
    double x = -leftController.getRawAxis(Constants.left_y_axis);
    return RobotContainer.to360(Math.toDegrees(Math.atan2(y,x)));
  }
  public double getRightAngle(){
    double y = -rightController.getRawAxis(Constants.left_x_axis);
    double x = -rightController.getRawAxis(Constants.left_y_axis);
    return RobotContainer.to360(Math.toDegrees(Math.atan2(y,x)));
  }
  public double getLeftMagnitude(){
    return Math.sqrt(Math.pow(leftController.getRawAxis(Constants.left_x_axis), 2) + Math.pow(leftController.getRawAxis(Constants.left_y_axis), 2));
  }
  public double getRightX(){
    return -rightController.getRawAxis(Constants.left_x_axis);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
