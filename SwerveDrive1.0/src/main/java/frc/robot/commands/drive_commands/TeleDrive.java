// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class TeleDrive extends CommandBase {
  /** Creates a new DriveTrain. */
  interface foCheck {void drive();}
  private SwerveDrive sd;
  private XboxController xc;
  public boolean fo = false;
  foCheck drive;
  /*
  double angle;
  double kp;
  double ki;
  double kd;
  double errorDiff = 0.1;
  
  PIDControl pid = new PIDControl(kp, ki, kd);
  */

  public TeleDrive(XboxController xc, SwerveDrive sd) {
    this.xc = xc;
    this.sd = sd;
    addRequirements(sd);
  }

  public void checkFO() {
    if (fo != false) {
      drive = () -> {sd.drive_fo(xc.getRawAxis(Constants.left_x_axis), xc.getRawAxis(Constants.left_y_axis), xc.getRawAxis(Constants.right_x_axis));};
    } else {
      drive = () -> {sd.drive(xc.getRawAxis(Constants.left_x_axis), xc.getRawAxis(Constants.left_y_axis), xc.getRawAxis(Constants.right_x_axis));};
    }
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    checkFO();
    //pid.setSetpoint(angle, NavXGyro.getAngle());
  }

  @Override
  public void execute() {
    drive.drive();
    double[] positions = sd.getPositions(0,4);
    double[] angles = sd.getAngles();

    SmartDashboard.putNumber("LF Angle", angles[0]);
    SmartDashboard.putNumber("LB Angle", angles[1]);
    SmartDashboard.putNumber("RF Angle", angles[2]);
    SmartDashboard.putNumber("RB Angle", angles[3]);

    SmartDashboard.putNumber("LF Pos", positions[0]);
    SmartDashboard.putNumber("LB Pos", positions[1]);
    SmartDashboard.putNumber("RF Pos", positions[2]);
    SmartDashboard.putNumber("RB Pos", positions[3]);
    SmartDashboard.putNumber("Navx Angle", sd.getAngle());
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  //@Override
  //public void execute() {
    // sd.drive(0, 0, pid.getOutput(NavXGyro.getAngle() * (RobotContainer.shouldTurnLeft(NavXGyro.getAngle(), angle)? 1:-1)));
  //}
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sd.stop();
  }
  
  // Returns true when the command should end.
  /*
  @Override
  public boolean isFinished() {
    return RobotContainer.angleDistance2(angle, NavXGyro.getAngle())<errorDiff;
  }
  */
}