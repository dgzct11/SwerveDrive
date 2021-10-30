// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.functional;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

public class Wheel {
  public TalonFX angle_m;
  public TalonFX speed_m;

  //dir index 0
  public double kpDir = 0.2;
  public double kiDir = 0.002;
  public double kdDir = 0;

  //thrust index 1 position
  public double kpThPos = 0.2;
  public double kiThPos = 0;
  public double kdThPos = 0;

  //thrust index 0 velocity
  public double kpTh = 0.01;
  public double kiTh = 0;
  public double kdTh = 0;
  public double kfTh = 0.045;

  /** Creates a new Wheel. */
  public Wheel(int angle_p, int speed_p) {
    angle_m = new TalonFX(angle_p);
    speed_m = new TalonFX(speed_p);

    angle_m.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    angle_m.configAllowableClosedloopError(0, 0.01 * Constants.units_per_degree);
    angle_m.setNeutralMode(NeutralMode.Brake); 
    angle_m.configMotionCruiseVelocity(500);
    angle_m.configMotionAcceleration(1000);

    speed_m.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    speed_m.configAllowableClosedloopError(0, 10, 0);
    setPID();

    angle_m.setSelectedSensorPosition(0);
    speed_m.setSelectedSensorPosition(0);
  }

  public void drive(double speed, double angle, double thrustCoefficients) {
    speed_m.set(TalonFXControlMode.PercentOutput, thrustCoefficients * speed * Constants.max_motor_percent);
    angle_m.set(TalonFXControlMode.Position, angle * Constants.units_per_degree);
  }

  public void drive_v(double speed, double angle, double thrustCoefficients) {
    speed_m.set(TalonFXControlMode.Velocity, thrustCoefficients * speed * Constants.talon_velocity_per_ms);
    angle_m.set(TalonFXControlMode.Position, angle * Constants.units_per_degree);
  }

  public void setPID() {
    angle_m.config_kP(0, kpDir);
    angle_m.config_kI(0, kiDir);
    angle_m.config_kD(0, kdDir);

    speed_m.config_kP(1, kpThPos);
    speed_m.config_kI(1, kiThPos);
    speed_m.config_kD(1, kdThPos);

    speed_m.config_kP(0, kpTh);
    speed_m.config_kI(0, kiTh);
    speed_m.config_kD(0, kdTh);
    speed_m.config_kF(0, kfTh);
  }
}
