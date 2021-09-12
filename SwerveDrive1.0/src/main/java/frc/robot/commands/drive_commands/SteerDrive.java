// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.XboxRemote;

public class SteerDrive extends CommandBase {
  /** Creates a new SteerDrive. */
  DriveTrain driveTrain;
  XboxRemote xbox;
  public SteerDrive(DriveTrain dt, XboxRemote xr) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    xbox = xr;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafeAngle = xbox.getLeftAngle();
      double speed = xbox.getLeftMagnitude();
      double circleRadius = 1/xbox.getRightX();
      driveTrain.arcDrive(strafeAngle, speed, circleRadius);
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
