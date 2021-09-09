// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class XboxRemote extends SubsystemBase {
  /** Creates a new XboxRemote. */
  XboxController xboxController;
  public XboxRemote(XboxController xc) {
    xboxController = xc;
  }

  public double getLeftAngle(){
    double y = -xboxController.getRawAxis(Constants.left_x_axis);
    double x = xboxController.getRawAxis(Constants.left_y_axis);
    return RobotContainer.to360(Math.toDegrees(Math.atan2(y,x)));
  }
  public double getLeftMagnitude(){
    return Math.sqrt(Math.pow(xboxController.getRawAxis(Constants.left_x_axis), 2) + Math.pow(xboxController.getRawAxis(Constants.left_y_axis), 2));
  }
  public double getRightX(){
    return xboxController.getRawAxis(Constants.right_x_axis);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
