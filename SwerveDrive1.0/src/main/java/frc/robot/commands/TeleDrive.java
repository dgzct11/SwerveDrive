// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.functional.SwerveDrive;

public class TeleDrive extends CommandBase {
  /** Creates a new DriveTrain. */
  private SwerveDrive sd;
  private XboxController xc;

  public TeleDrive(XboxController xc, SwerveDrive sd) {
    this.xc = xc;
    this.sd = sd;
  }

  public void drive() {
    sd.drive(xc.getRawAxis(Constants.left_x_axis), xc.getRawAxis(Constants.left_y_axis), xc.getRawAxis(Constants.right_x_axis));
  }

  @Override
  public void execute() {
    drive();
    double[] positions = sd.getPositions(0,4);
    double[] angles = sd.getAngles();

    SmartDashboard.putNumber("LF Angle", angles[0]);
    SmartDashboard.putNumber("LB Angle", angles[1]);
    SmartDashboard.putNumber("RF Angle", angles[2]);
    SmartDashboard.putNumber("RB Angle", angles[3]);

    SmartDashboard.putNumber("LF Pos", positions[0]);
    SmartDashboard.putNumber("LB Pos", positions[1]);
    SmartDashboard.putNumber("RF Pos", positions[2]);
    SmartDashboard.putNumber("RB Pos", positions[3]);
  }
}