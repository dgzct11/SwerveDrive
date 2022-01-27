// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.functional.PIDControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.XboxRemote;

public class AlignWithObject extends CommandBase {
  /** Creates a new AlignWithObject. */
  double angle;
  double kp = 0.02;
  double ki = 0;
  double kd = 0;
  double errorDiff = 0.01;
  DriveTrain driveTrian;
  PIDControl pid = new PIDControl(kp, ki, kd);
  LimeLight limelight;
  XboxRemote xboxRemote;
  public AlignWithObject(DriveTrain dt, LimeLight lt, XboxRemote xr) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrian = dt;
    limelight = lt;
   xboxRemote = xr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.in_auto = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double error = 0.6;
    if(limelight.inView()){
      error =Math.min( kp*(Math.abs(limelight.getHorizontalAngleDiff())),0.3);
       driveTrian.rotateDriveVelocity(0, 0, error * (limelight.getHorizontalAngleDiff()>0 ? -1:1));
    } 
    double angle = xboxRemote.getLeftAngle();
      double speed = xboxRemote.getLeftMagnitude();
     driveTrian.rotateDriveVelocity(angle, speed, error * (limelight.getHorizontalAngleDiff()>0 ? -1:1));
       
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.in_auto = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
