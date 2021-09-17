// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import frc.robot.functional.Position;
import frc.robot.functional.Trajectory;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Odometry;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */
  
  DriveTrain driveTrain;
  Odometry odometry;
  Trajectory trajectory;
  double acceleration = 2, velocity = 5;
  double previousTime;
  double initialTime;
  public FollowTrajectory(double[][] points, double[] distances, DriveTrain dt, Odometry od) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    odometry = od;
    trajectory = new Trajectory(points, distances, acceleration, velocity);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odometry.reset();
    initialTime = System.currentTimeMillis()/1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Position currentPosition = odometry.getPosition();
    double time = System.currentTimeMillis()/1000 - initialTime;
    double timeUnit = time-previousTime;
    Position newPos = trajectory.getPosition(time+timeUnit);
    double[] start = {currentPosition.x, currentPosition.y};
    double[] end = {newPos.x, newPos.y};
    double angleToPoint = RobotContainer.angleToPoint(start, end);
    double currentAngle = currentPosition.angle;
    double speed = RobotContainer.distance(start, end)/timeUnit;
    driveTrain.rotateDrive((angleToPoint-currentAngle+360)%360, speed, 0);
    previousTime = time;
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
