// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.functional.SwerveDrive;
import frc.robot.functional.Wheel;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.TeleDrive;

public class DriveManager extends CommandBase {
  private Wheel bl = new Wheel(Constants.bl_angle, Constants.bl_speed);
  private Wheel br = new Wheel(Constants.br_angle, Constants.br_speed);
  private Wheel fr = new Wheel(Constants.fr_angle, Constants.fr_speed);
  private Wheel fl = new Wheel(Constants.fl_angle, Constants.fl_speed);

  public SwerveDrive sd = new SwerveDrive(br, bl, fr, fl);
  /** Creates a new DriveManager. */
  private TeleDrive td;
  private AutoDrive ad;

  public DriveManager(XboxController xc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.td = new TeleDrive(xc, sd);
    this.ad = new AutoDrive(sd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Create a TeleDrive and AutoDrive subsystems where we can call each one from
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (Constants.drive_mode) {
      case 0:
        td.drive();
        break;
      case 1:
        ad.drive();
        break;
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
