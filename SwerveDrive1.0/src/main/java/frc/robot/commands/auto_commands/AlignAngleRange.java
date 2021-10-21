// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.functional.PIDControl;
import frc.robot.subsystems.LimeLight;

public class AlignAngleRange extends CommandBase {
  /** Creates a new AlignWithObject. */
  double angle;
  double kp = 0.03;

  double kpRange = 0.3;
  double errorDiff = 0.01;
  DriveTrain driveTrain;

  LimeLight limelight;
  double distance = 2;
  double area = 0.0316;

  public AlignAngleRange(DriveTrain dt, LimeLight lt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
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
     double strafeAngle = limelight.getHorizontalAngleDiff();
     double speed = Math.min( Math.abs(limelight.getDistanceFromArea(area)-distance)*kpRange, 0.3) * (distance > limelight.getDistanceFromArea(area) ? -1:1);
     double angleError = Math.min( kp * (Math.abs(limelight.getHorizontalAngleDiff())),0.3) * (limelight.getHorizontalAngleDiff()>0 ? -1:1);
     SmartDashboard.putNumber("Speed Align", speed);
     driveTrain.rotateDrive(strafeAngle, speed, angleError);
    }
    else{
      driveTrain.rotateDrive(0, 0, 0.3);
     
    }
  }
  boolean somethign = true;
  int val =  (somethign == true ? 0: 100);
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
