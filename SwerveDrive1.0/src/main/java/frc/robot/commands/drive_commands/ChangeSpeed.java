// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeSpeed extends InstantCommand {
  double increment = 0.5;
  public ChangeSpeed(double i) {
    // Use addRequirements() here to declare subsystem dependencies.
     increment = i;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    Constants.velocityMax = Math.max(0, Constants.velocityMax+increment);
  }
}
