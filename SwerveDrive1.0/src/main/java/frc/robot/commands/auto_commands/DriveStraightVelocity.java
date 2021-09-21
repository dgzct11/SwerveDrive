// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveStraightVelocity extends CommandBase {
  /** Creates a new DriveStraightVelocity. */
  DriveTrain driveTrain;
  double time;
  double initialTime = 0;
  public DriveStraightVelocity(DriveTrain dt, double t) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    time = t;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] velocities = {1,1,1,1};
    driveTrain.setThrustVelocity(velocities);
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
