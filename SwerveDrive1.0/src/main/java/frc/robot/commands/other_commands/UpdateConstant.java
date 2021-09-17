// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.other_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class UpdateConstant extends CommandBase {
  /** Creates a new DisplayDashboard. */
  //TODO
  DriveTrain driveTrain;
  String constant;
  double amount;
  boolean changeDir;
  public UpdateConstant(DriveTrain dt, String str, double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    constant = str;
    amount = a;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if(constant.equals("kpDir")) driveTrain.kpDir += amount;
      else if(constant.equals("kiDir")) driveTrain.kiDir += amount;
      else if(constant.equals("kdDir")) driveTrain.kdDir += amount;
      else if(constant.equals("kfDir")) driveTrain.kfDir += amount;
      else if(constant.equals("kpTh")) driveTrain.kpTh += amount;
      else if(constant.equals("kiTh")) driveTrain.kiTh += amount;
      else if(constant.equals("kdTh")) driveTrain.kdTh += amount;
      else if(constant.equals("kfTh")) driveTrain.kfTh += amount;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
