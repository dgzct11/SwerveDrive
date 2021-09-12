// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NavXGyro extends SubsystemBase {
  /** Creates a new NavXGyro. */
  //TODO
  public AHRS ahrs = new AHRS(spi_port_id);
  public NavXGyro() {}

  public void getAngle(){
    return RobotContainer.navxTo360(ahrs.getAngle());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
