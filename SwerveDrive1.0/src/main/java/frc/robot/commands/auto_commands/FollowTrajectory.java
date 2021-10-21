// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.functional.Position;
import frc.robot.functional.SwerveDrive;
import frc.robot.functional.Trajectory;
import frc.robot.subsystems.Odometry;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */
  Odometry odometry;
  Trajectory trajectory;
  double acceleration = 1, velocity = 2;
  double previousTime;
  double initialTime;
  double timeUnit = 1;
  SwerveDrive sd;

  public FollowTrajectory(double[][] points, double[] distances, SwerveDrive sd, Odometry od) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sd = sd;
    odometry = od;
    trajectory = new Trajectory(points, distances, acceleration, velocity);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.in_auto = true;
    odometry.reset();
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
    SmartDashboard.putNumber("End X", end[0]);
    SmartDashboard.putNumber("End Y", end[1]);
    SmartDashboard.putNumber("current time", System.currentTimeMillis()/1000. - initialTime);
    SmartDashboard.putNumber("totalTime", trajectory.totalTime);
    double angleToPoint = RobotContainer.angleToPoint(start, end);
    double currentAngle = currentPosition.angle;
    double speed = RobotContainer.distance(start, end)/timeUnit;
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("angleToPoint", angleToPoint);
    //sd.drive(angleToPoint, speed, 0);
    previousTime = time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.in_auto = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  (System.currentTimeMillis()/1000 - initialTime )>=trajectory.totalTime;
  }
}
