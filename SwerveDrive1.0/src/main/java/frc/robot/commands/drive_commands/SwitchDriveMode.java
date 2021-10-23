// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.XboxRemote;

public class SwitchDriveMode extends CommandBase {
  /** Creates a new Switch. */
  DriveTrain driveTrain;
  XboxRemote xboxRemote;
  public SwitchDriveMode(DriveTrain dt, XboxRemote xr) {
    // Use addRequirements() here to declare subsystem dependencies.
    xboxRemote = xr;
    driveTrain = dt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Constants.drive_mode == 0){
      FieldOriented fo = new FieldOriented(driveTrain, xboxRemote);
      fo.addRequirements(driveTrain);
      driveTrain.setDefaultCommand(fo);

      Constants.drive_mode = 1;
    }
    if(Constants.drive_mode == 1){
      SwerveDrive fo = new SwerveDrive(driveTrain, xboxRemote);
      fo.addRequirements(driveTrain);
      driveTrain.setDefaultCommand(fo);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
