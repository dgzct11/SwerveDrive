// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.functional.Wheel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/** An example command that uses an example subsystem. */
public class SwerveDrive extends SubsystemBase{
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

  private Wheel br;
  private Wheel bl;
  private Wheel fr;
  private Wheel fl;
  private Wheel[] wheels = {br, bl, fr, fl};
  
  public int[] thrustCoefficients = {1,1,1,1};

  public SwerveDrive (Wheel br, Wheel bl, Wheel fr, Wheel fl) {
    this.br = br;
    this.bl = bl;
    this.fr = fr;
    this.fl = fl;
    
    br.speed_m.setInverted(true);
    fr.speed_m.setInverted(true);
  }

  public double[][] trig (double x1, double y1, double x2) {
    double sine = Constants.side_over_radius;

    double a = x1 - x2 * sine;
    double b = x1 + x2 * sine;
    double c = y1 - x2 * sine;
    double d = y1 + x2 * sine;

    double brSpeed = Math.sqrt ((b * b) + (d * d));
    double blSpeed = Math.sqrt ((b * b) + (c * c));
    double frSpeed = Math.sqrt ((a * a) + (d * d));
    double flSpeed = Math.sqrt ((a * a) + (c * c));

    double brAngle = Math.atan2 (b, d) * 180 / Math.PI;
    double blAngle = Math.atan2 (b, c) * 180 / Math.PI;
    double frAngle = Math.atan2 (a, d) * 180 / Math.PI;
    double flAngle = Math.atan2 (a, c) * 180 / Math.PI;

    double[][] dir = {{brSpeed, blSpeed, frSpeed, flSpeed},{brAngle, blAngle, frAngle, flAngle}};

    return dir;
  }

  public void drive (double x1, double y1, double x2) {
    double[][] dir = trig(x1, y1, x2);
    for (short i = 0; i < 4; i++) {
      wheels[i].drive(dir[0][i], dir[1][i]);
    }

    double[] angles = getAngles();
    SmartDashboard.putNumber("LF turnto", angles[0]);
    SmartDashboard.putNumber("LB turnto", angles[1]);
    SmartDashboard.putNumber("RF turnto", angles[2]);
    SmartDashboard.putNumber("RB turnto", angles[3]);
  }

  public void drive_fo (double x1, double y1, double x2) {
    y1 *= -1;
    double currentAngle = NavXGyro.getAngle();
    y1 = y1 * Math.cos(currentAngle) + x1 * Math.sin(currentAngle);
    x1 = -y1 * Math.sin(currentAngle) + x1 * Math.cos(currentAngle);
    double[][] dir = trig(x1, y1, x2);
    for (short i = 0; i < 4; i++) {
      wheels[i].drive(dir[0][i], dir[1][i]);
    }

    double[] angles = getAngles();
    SmartDashboard.putNumber("LF turnto", angles[0]);
    SmartDashboard.putNumber("LB turnto", angles[1]);
    SmartDashboard.putNumber("RF turnto", angles[2]);
    SmartDashboard.putNumber("RB turnto", angles[3]);
  }

  public void driveDirectionalAngles(double[] angles) {
    double[] currentAngles = getAngles();
    SmartDashboard.putNumber("LF Diff", RobotContainer.angleDistance2(currentAngles[0], angles[0]));
    SmartDashboard.putNumber("LB Diff", RobotContainer.angleDistance2(currentAngles[1], angles[1]));
    SmartDashboard.putNumber("RF Diff", RobotContainer.angleDistance2(currentAngles[2], angles[2]));
    SmartDashboard.putNumber("RB Diff", RobotContainer.angleDistance2(currentAngles[3], angles[3]));
    for(int i = 0; i<4; i++) {
      wheels[i].drive(0, (wheels[i].angle_m.getSelectedSensorPosition() +
      RobotContainer.angleDistance2(angles[i], currentAngles[i])*Constants.units_per_degree *
      (RobotContainer.shouldTurnLeft(currentAngles[i], angles[i]) ? 1 : -1)));
    }
  }

  public void setDirectionalAnglesEff(double[] angles) {
    double[] currentAngles = getAngles();

    for (int i = 0; i < 4; i++) {
      if (RobotContainer.angleDistance2(angles[i], currentAngles[i]) > 90) {
        thrustCoefficients[i] = -1;
        angles[i] = (angles[i] + 180) % 360;
      } else
        thrustCoefficients[i] = 1;
      wheels[i].drive(0, 
      ((wheels[i].angle_m.getSelectedSensorPosition() + RobotContainer.angleDistance2(angles[i], currentAngles[i])
              * Constants.units_per_degree * (RobotContainer.shouldTurnLeft(currentAngles[i], angles[i]) ? 1 : -1))));
    }
    SmartDashboard.putNumber("LF Diff", RobotContainer.angleDistance2(currentAngles[0], angles[0]));
    SmartDashboard.putNumber("LB Diff", RobotContainer.angleDistance2(currentAngles[1], angles[1]));
    SmartDashboard.putNumber("RF Diff", RobotContainer.angleDistance2(currentAngles[2], angles[2]));
    SmartDashboard.putNumber("RB Diff", RobotContainer.angleDistance2(currentAngles[3], angles[3]));
  }

  public void setPID() {
    
    kpDir = kpDirEntry.getDouble(kpDir);
    kiDir = kiDirEntry.getDouble(kiDir);
    kdDir = kdDirEntry.getDouble(kdDir);

    kpTh = kpThEntry.getDouble(kpTh);
    kiTh = kiThEntry.getDouble(kiTh);
    kdTh = kdThEntry.getDouble(kdTh);
    kfTh = kfThEntry.getDouble(kfTh);

    for (Wheel el:wheels) {
      el.angle_m.config_kP(slotIdx, kpDir);
      el.angle_m.config_kI(slotIdx, kiDir);
      el.angle_m.config_kD(slotIdx, kdDir);

      el.speed_m.config_kP(slotIdx, kpTh);
      el.speed_m.config_kI(slotIdx, kiTh);
      el.speed_m.config_kD(slotIdx, kdTh);
      el.speed_m.config_kF(slotIdx, kfTh);
    }
  }

  public Wheel[] getWheels() {
    return wheels;
  }

  public void stop() {
    for(Wheel el:wheels) {el.drive(0, 0);}
  }

  public double[] getPositions(int first, int last ) {
    double[] positions = new double[8];
    for (;first < last; first++) {
      if (first<4) {
        positions[first] = wheels[first].speed_m.getSelectedSensorPosition();
      } else {
        positions[first] = wheels[first-4].angle_m.getSelectedSensorPosition();
      }
    }
    return positions;
  }

  public double[] getAngles() {
    double[] angles = getPositions(4,8);
    for (int i = 0; i < angles.length; i++) {
      angles[i] = RobotContainer.floorMod(angles[i] / Constants.units_per_degree, 360);
    }
    return angles;
  }
}
