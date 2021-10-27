// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.XboxRemote;

public class FieldOriented extends CommandBase {
  /** Creates a new SwerveDrive. */
  DriveTrain driveTrain;
  XboxRemote xbox;

  public FieldOriented(DriveTrain dt, XboxRemote xr) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    xbox = xr;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(!Constants.in_auto){
      double strafeAngle = xbox.getLeftAngle();
      SmartDashboard.putNumber("Strafe Angle", strafeAngle);
      double speed = xbox.getLeftMagnitude();
      double rotateSpeed = xbox.getRightX();
      strafeAngle = (RobotContainer.angleDistance2(NavXGyro.getAngle(), strafeAngle) * (RobotContainer.shouldTurnLeft(NavXGyro.getAngle(), strafeAngle)?1:-1) + 360 ) %360;
      driveTrain.rotateDriveVelocity(strafeAngle, speed, rotateSpeed);
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
