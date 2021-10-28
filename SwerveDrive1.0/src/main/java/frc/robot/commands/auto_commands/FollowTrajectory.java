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
  double acceleration = 1, velocity = 1;
  double previousTime;
  double initialTime;
  double timeUnit = 0.1;
  public FollowTrajectory(double[][] points, double[] distances, DriveTrain dt, Odometry od) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    odometry = od;
    trajectory = new TrajectoryCircleLine(points, distances, acceleration, velocity);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.in_auto = true;
    odometry.reset();
    NavXGyro.ahrs.reset();
    initialTime = System.currentTimeMillis()/1000.;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Position currentPosition = odometry.getPosition();
    double time = System.currentTimeMillis()/1000. - initialTime;
   
    Position newPos = trajectory.getPosition(time+timeUnit);
    double[] start = {currentPosition.x, currentPosition.y};
    double[] end = {newPos.x, newPos.y};
   
   
    double angleToPoint = RobotContainer.angleToPoint(start, end);
 
    double speed = RobotContainer.distance(start, end)/timeUnit;
    
    driveTrain.rotateDriveVelocity((angleToPoint-NavXGyro.ahrs.getAngle() + 360)%360, speed, 0);
    previousTime = time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double[] currentPosition = odometry.getPosition().point;
    double[] newPos = trajectory.getEndPoint().point;
    double angleToPoint = RobotContainer.angleToPoint(currentPosition, newPos);
    double distance = RobotContainer.distance(currentPosition, newPos);

    driveTrain.driveDistance(angleToPoint, distance);
    Constants.in_auto = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  (System.currentTimeMillis()/1000 - initialTime )>=trajectory.totalTime;
  }
}
