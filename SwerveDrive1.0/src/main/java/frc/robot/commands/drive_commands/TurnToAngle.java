// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.functional.PIDControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */
  double angle;
  double kp;
  double ki;
  double kd;
  double errorDiff = 0.1;
  DriveTrain driveTrian;
  PIDControl pid = new PIDControl(kp, ki, kd);
  public TurnToAngle(DriveTrain dt, double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrian = dt;
    angle = a;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(angle, NavXGyro.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrian.rotateDriveVelocity(0, 0, pid.getOutput(NavXGyro.getAngle() * (RobotContainer.shouldTurnLeft(NavXGyro.getAngle(), angle)? 1:-1)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrian.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.angleDistance2(angle, NavXGyro.getAngle())<errorDiff;
  }
}
