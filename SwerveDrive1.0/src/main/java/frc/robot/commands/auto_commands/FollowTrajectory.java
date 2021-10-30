// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;

import frc.robot.functional.trajectory.Position;
import frc.robot.functional.trajectory.TrajectoryCircleLine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Odometry;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */
  
  DriveTrain driveTrain;
  Odometry odometry;
  TrajectoryCircleLine trajectory;
  
  double previousTime;
  double initialTime;
  double timeUnit = 0.1;
  double finalAngle = 0;
  public FollowTrajectory(TrajectoryCircleLine t, DriveTrain dt, Odometry od, double fa) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    odometry = od;
    trajectory = t;
    finalAngle = fa;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.in_auto = true;
    odometry.reset();
    NavXGyro.ahrs.reset();
    initialTime = System.currentTimeMillis()/1000.;
    Constants.velocityMax = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] currentPosition = odometry.currentPosition.point;
    double time = System.currentTimeMillis()/1000. - initialTime;
    double[] newPos = trajectory.getPosition(time+timeUnit).point;
    double angleToPoint = RobotContainer.angleToPoint(currentPosition, newPos);
    double speed = RobotContainer.distance(currentPosition, newPos)/timeUnit; 
    double angle = trajectory.getCurrentAngle();
    
    driveTrain.alignDrive(angleToPoint, speed, angle);
    previousTime = time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double[] currentPosition = odometry.getPosition().point;
    double[] newPos = trajectory.getEndPoint().point;
    double angleToPoint = RobotContainer.angleToPoint(currentPosition, newPos);
    double distance = RobotContainer.distance(currentPosition, newPos);

    //driveTrain.driveDistance(angleToPoint, distance);
    Constants.in_auto = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  (System.currentTimeMillis()/1000 - initialTime )>=trajectory.totalTime;
  }
}



