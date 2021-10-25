// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.XboxRemote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwitchDriveMode extends InstantCommand {
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
      Constants.drive_mode = 0;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

