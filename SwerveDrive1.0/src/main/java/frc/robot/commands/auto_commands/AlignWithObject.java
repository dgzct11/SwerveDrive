// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.functional.PIDControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class AlignWithObject extends CommandBase {
  /** Creates a new AlignWithObject. */
  double angle;
  double kp = 0.03;
  double ki = 0;
  double kd = 0;
  double errorDiff = 0.01;
  DriveTrain driveTrian;
  PIDControl pid = new PIDControl(kp, ki, kd);
  LimeLight limelight;
  public AlignWithObject(DriveTrain dt, LimeLight lt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrian = dt;
    limelight = lt;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.in_auto = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    if(limelight.inView()){
     double error =Math.min( kp*(Math.abs(limelight.getHorizontalAngleDiff())),0.3);
       driveTrian.rotateDriveVelocity(0, 0, error * (limelight.getHorizontalAngleDiff()>0 ? 1:-1));
    }
    else{
      driveTrian.rotateDriveVelocity(0, 0, 0.3);
       }
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