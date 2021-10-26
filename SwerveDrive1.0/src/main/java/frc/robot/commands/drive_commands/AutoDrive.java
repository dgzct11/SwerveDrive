// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.functional.LimeLight;
import frc.robot.subsystems.SwerveDrive;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  interface autodmCheck {void run();};
  public int adm = 0;
  autodmCheck drive;
  autodmCheck periodic;
  SwerveDrive sd;
  LimeLight ll;

  public AutoDrive(SwerveDrive sd) {
    this.sd = sd;
    ll = new LimeLight(sd);
    addRequirements(sd);
  }

  public void checkAutoDM() {
    switch (adm) {
      case 0:
        drive = () -> {ll.trackBall();};
        periodic = () -> {ll.updateValues();};
        break;
    }
  }

  @Override
  public void initialize() {
    checkAutoDM();
    //pid.setSetpoint(angle, NavXGyro.getAngle());
  }

  @Override
  public void execute() {
    drive.run();
    periodic.run();
  }

  @Override
  public void end(boolean interrupted) {
    sd.stop();
  }
}
