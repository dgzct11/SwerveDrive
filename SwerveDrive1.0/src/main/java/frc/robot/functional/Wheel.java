// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.functional;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Wheel {
  public TalonFX angle_m;
  public TalonFX speed_m;

  private ShuffleboardTab tab = Shuffleboard.getTab("PID DriveTrain Constants");
  
  private NetworkTableEntry kpDirEntry = tab.add("Directional KP", 0).getEntry();
  private NetworkTableEntry kiDirEntry = tab.add("Directional KI", 0).getEntry();
  private NetworkTableEntry kdDirEntry = tab.add("Directional KD", 0).getEntry();

  private NetworkTableEntry kpThEntry = tab.add("Thrust KP", 0).getEntry();
  private NetworkTableEntry kiThEntry = tab.add("Thrust KI", 0).getEntry();
  private NetworkTableEntry kdThEntry = tab.add("Thrust KD", 0).getEntry();
  private NetworkTableEntry kfThEntry = tab.add("Thrust KF", 0).getEntry();

  public double kpDir = 0.2;
  public double kiDir = 0.002;
  public double kdDir = 0;
  public double kfDir = 0;
  public int slotIdx = 0;

  public double kpTh = 0.01;
  public double kiTh = 0;
  public double kdTh = 0;
  public double kfTh = 0.045;

  /** Creates a new Wheel. */
  public Wheel(int angle_p, int speed_p) {
    angle_m = new TalonFX(angle_p);
    speed_m = new TalonFX(speed_p);

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

  public void drive(double speed, double angle, double thrustCoefficients) {
    speed_m.set(TalonFXControlMode.PercentOutput, thrustCoefficients * speed * Constants.max_motor_percent);
    angle_m.set(TalonFXControlMode.Position, angle);
  }

  public void drive_v(double speed, double angle, double thrustCoefficients) {
    speed_m.set(TalonFXControlMode.Velocity, thrustCoefficients * speed * Constants.talon_velocity_per_ms);
    angle_m.set(TalonFXControlMode.Position, angle * Constants.units_per_degree);
  }

  public void setPID() {
    kpDir = kpDirEntry.getDouble(kpDir);
    kiDir = kiDirEntry.getDouble(kiDir);
    kdDir = kdDirEntry.getDouble(kdDir);

    kpTh = kpThEntry.getDouble(kpTh);
    kiTh = kiThEntry.getDouble(kiTh);
    kdTh = kdThEntry.getDouble(kdTh);
    kfTh = kfThEntry.getDouble(kfTh);

    angle_m.config_kP(slotIdx, kpDir);
    angle_m.config_kI(slotIdx, kiDir);
    angle_m.config_kD(slotIdx, kdDir);

    speed_m.config_kP(slotIdx, kpTh);
    speed_m.config_kI(slotIdx, kiTh);
    speed_m.config_kD(slotIdx, kdTh);
    speed_m.config_kF(slotIdx, kfTh);
  }
}
