// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  SwerveDrive sd;
  public AutoDrive(SwerveDrive sd) {
    this.sd = sd;
    addRequirements(sd);
  }

  public void drive() {
    
  }

  @Override
  public void execute() {
    drive();
    // This method will be called once per scheduler run
  }
}
