// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnWheelsToAngle extends CommandBase {
  /** Creates a new TurnWheelsToAngle. */
  DriveTrain driveTrain;
  double angle;
  public TurnWheelsToAngle(DriveTrain dt, double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    angle = a;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] angles = {angle, angle, angle,angle};
    driveTrain.setDirectionalAngles(angles);
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
