// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Odometry;

public class CatchBall extends CommandBase {
  /** Creates a new CatchBall. */
  DriveTrain driveTrain;
  Odometry odometry;
  LimeLight limelight;
  double kp;
  public CatchBall(DriveTrain dt, Odometry od, LimeLight l) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    odometry = od;
    limelight = l;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.in_auto = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] targetPointTime = limelight.trackBall();
    
    double speed = RobotContainer.distance(targetPointTime, odometry.currentPosition.point)/targetPointTime[2];
    double strafeAngle = RobotContainer.angleToPoint(odometry.currentPosition.point, targetPointTime);
    double angleError = Math.min( kp * (Math.abs(limelight.getHorizontalAngleDiff())),0.3) * (limelight.getHorizontalAngleDiff()>0 ? -1:1);
   
    driveTrain.rotateDriveVelocity((strafeAngle-NavXGyro.ahrs.getAngle() + 360)%360, speed, angleError);
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
