// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.functional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class Wheel {
  public TalonSRX angle_m;
  public TalonSRX speed_m;
  public int slotIdx = 0;
  public double kpDir=0;
  public double kiDir=0;
  public double kdDir=0;
  public double kfDir=0.5;

  /** Creates a new Wheel. */
  public Wheel(int angle_p, int speed_p) {
    angle_m = new TalonSRX(angle_p);
    speed_m = new TalonSRX(speed_p);

    angle_m.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    angle_m.configAllowableClosedloopError(slotIdx, 1 * Constants.units_per_degree);
    angle_m.setNeutralMode(NeutralMode.Brake); 
    angle_m.configMotionCruiseVelocity(100);
    angle_m.configMotionAcceleration(100);
    angle_m.config_kP(slotIdx, kpDir);
    angle_m.config_kI(slotIdx, kiDir);
    angle_m.config_kD(slotIdx, kdDir);
    angle_m.config_kF(slotIdx, kfDir);

    speed_m.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    angle_m.setSelectedSensorPosition(0);
    speed_m.setSelectedSensorPosition(0);
  }

  public void drive(double speed, double angle) {
    speed_m.set(ControlMode.PercentOutput, speed * Constants.max_motor_percent);
    angle_m.set(ControlMode.Position, angle * Constants.units_per_degree);
  }

  public void drive_v(double speed, double angle) {
    speed_m.set(ControlMode.Velocity, speed * Constants.talon_velocity_per_ms);
    angle_m.set(ControlMode.Position, angle * Constants.units_per_degree);
  }
}
