// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.other_commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.XboxRemote;

import frc.robot.functional.Position;
public class DisplayDashboard extends CommandBase {
  /** Creates a new DisplayDashboard. */
  //TODO
  DriveTrain driveTrain;
  Odometry od;
  public DisplayDashboard(DriveTrain dt, XboxRemote xr, Odometry o) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    od = o;
    addRequirements(xr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wheel angles
    double[] angles = driveTrain.getAngles();
    SmartDashboard.putNumber("LF Angle", angles[0]);
    SmartDashboard.putNumber("LB Angle", angles[1]);
    SmartDashboard.putNumber("RF Angle", angles[2]);
    SmartDashboard.putNumber("RB Angle", angles[3]);

    //wheel angle positions
    double[] dirPositions = driveTrain.getDirectionalPositions();
    SmartDashboard.putNumber("LB Directional Position", dirPositions[1]);
    SmartDashboard.putNumber("LF Directional Position", dirPositions[0]);
    SmartDashboard.putNumber("RF Directional Position", dirPositions[2]);
    SmartDashboard.putNumber("RB Directional Position", dirPositions[3]);

    //thrust positions
    double[] thrustPositions = driveTrain.getThrustPositions();
    SmartDashboard.putNumber("LF Thrust Position", thrustPositions[0]);
    SmartDashboard.putNumber("LB Thrust Position", thrustPositions[1]);
    SmartDashboard.putNumber("RF Thrust Position", thrustPositions[2]);
    SmartDashboard.putNumber("RB Thrust Position", thrustPositions[3]);

    //Odometry variables
    Position currentPosition = od.currentPosition;
    SmartDashboard.putNumber("Odometry X", currentPosition.x);
    SmartDashboard.putNumber("Odometry Y", currentPosition.y);
    SmartDashboard.putNumber("Odometry Angle", currentPosition.angle);

    //NavX Variables
    SmartDashboard.putNumber(" Navx X", NavXGyro.ahrs.getDisplacementX());
    SmartDashboard.putNumber("Navx Y", NavXGyro.ahrs.getDisplacementY());
    SmartDashboard.putNumber("Navx Angle", NavXGyro.getAngle());
    /*SmartDashboard.putNumber("KP", driveTrain.kpDir);
    SmartDashboard.putNumber("KI", driveTrain.kiDir);
    SmartDashboard.putNumber("KD", driveTrain.kdDir);
    SmartDashboard.putNumber("KF", driveTrain.kfDir);
*/
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
