// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.functional.PIDControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Odometry;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */
  DriveTrain driveTrain;
  Odometry od;
  double angle;
  double kp;
  double ki = 0;
  double kd = 0;
  PIDControl pid;
  double errorThreshold = 0.001;
  public TurnToAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
    pid = new PIDControl(kp, ki, kd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(angle, od.currentPosition.angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.rotateDrive(0, 0, (RobotContainer.shouldTurnLeft(od.currentPosition.angle, angle)?1:-1)*pid.getOutputFromError(RobotContainer.angleDistance2(od.currentPosition.angle, angle)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.getOutputFromError(RobotContainer.angleDistance2(od.currentPosition.angle, angle))<errorThreshold;
  }
}
