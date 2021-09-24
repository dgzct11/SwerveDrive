// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.functional.PIDControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class AlignWithObject extends CommandBase {
  /** Creates a new AlignWithObject. */
  double angle;
  double kp;
  double ki;
  double kd;
  double errorDiff = 0.1;
  DriveTrain driveTrian;
  PIDControl pid = new PIDControl(kp, ki, kd);
  LimeLight limelight;
  public AlignWithObject(DriveTrain dt, LimeLight lt, double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrian = dt;
    limelight = lt;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.inView()){
      pid.setSetpoint(0, limelight.getHorizontalAngleDiff() );
      driveTrian.rotateDrive(0, 0, pid.getOutput(limelight.getHorizontalAngleDiff()) * (limelight.getHorizontalAngleDiff()>0 ? -1:1));
    }
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
