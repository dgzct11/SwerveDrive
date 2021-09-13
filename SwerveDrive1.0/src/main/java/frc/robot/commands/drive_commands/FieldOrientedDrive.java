// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.functional.PIDControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.XboxRemote;

public class FieldOrientedDrive extends CommandBase {
  /** Creates a new FieldOrientedDrive. */
  XboxRemote xbox;
  DriveTrain driveTrain;
  PIDControl pid = new PIDControl(0.00, 00, 0);
  public FieldOrientedDrive(DriveTrain dt, XboxRemote xr) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    xbox = xr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafeAngle = xbox.getLeftAngle();
    double speed = xbox.getLeftMagnitude();
    double rotateAngle = xbox.getRightAngle();
    pid.setSetpoint(rotateAngle, NavXGyro.getAngle());

    driveTrain.rotateDrive(strafeAngle, speed, pid.getOutput(NavXGyro.getAngle()) * (RobotContainer.shouldTurnLeft(NavXGyro.getAngle(), rotateAngle)?1:-1));
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
