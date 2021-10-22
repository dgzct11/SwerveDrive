// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.drive_commands.AutoDrive;
import frc.robot.commands.drive_commands.TeleDrive;
import frc.robot.functional.Wheel;
import frc.robot.subsystems.SwerveDrive;

public class DriveManager extends CommandBase {
  public int drivemode;
  private XboxController xc;
  private Wheel bl = new Wheel(Constants.bl_angle, Constants.bl_speed);
  private Wheel br = new Wheel(Constants.br_angle, Constants.br_speed);
  private Wheel fr = new Wheel(Constants.fr_angle, Constants.fr_speed);
  private Wheel fl = new Wheel(Constants.fl_angle, Constants.fl_speed);

  private SwerveDrive sd = new SwerveDrive(br, bl, fr, fl);
  private TeleDrive td = new TeleDrive(xc, sd);
  private AutoDrive ad = new AutoDrive(sd);
  /** Creates a new DriveManager. */
  public DriveManager(XboxController xc) {
    this.xc = xc;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (drivemode) {
      case 0:
        sd.setDefaultCommand(td);
        break;
      case 1:
        sd.setDefaultCommand(td);
        td.fo = true;
        break;
      case 2:
        sd.setDefaultCommand(ad);
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
