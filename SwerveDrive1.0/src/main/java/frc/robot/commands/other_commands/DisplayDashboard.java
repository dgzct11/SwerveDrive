// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.other_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.XboxRemote;

public class DisplayDashboard extends CommandBase {
  /** Creates a new DisplayDashboard. */
  //TODO
  DriveTrain driveTrain;
  public DisplayDashboard(DriveTrain dt, XboxRemote xr) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(xr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] angles = driveTrain.getAngles();
    SmartDashboard.putNumber("LF Angle", angles[0]);
    SmartDashboard.putNumber("LB Angle", angles[1]);
    SmartDashboard.putNumber("RF Angle", angles[2]);
    SmartDashboard.putNumber("RB Angle", angles[3]);
    double[] dirPositions = driveTrain.getDirectionalPositions();
    SmartDashboard.putNumber("LF Position", dirPositions[0]);
    SmartDashboard.putNumber("LB Position", dirPositions[1]);
    SmartDashboard.putNumber("RF Position", dirPositions[2]);
    SmartDashboard.putNumber("RB Position", dirPositions[3]);

    /*SmartDashboard.putNumber("KP", driveTrain.kpDir);
    SmartDashboard.putNumber("KI", driveTrain.kiDir);
    SmartDashboard.putNumber("KD", driveTrain.kdDir);
    SmartDashboard.putNumber("KF", driveTrain.kfDir);
*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
