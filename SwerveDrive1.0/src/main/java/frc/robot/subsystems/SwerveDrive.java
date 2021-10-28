// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.functional.Wheel;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private Wheel[] wheels = new Wheel[4];
  public int[] thrustCoefficients = {1,1,1,1};
  public static AHRS ahrs = new AHRS(Constants.mxp_port);

  public SwerveDrive (Wheel br, Wheel bl, Wheel fr, Wheel fl) {
    ahrs.reset();
    ahrs.zeroYaw();

    wheels[0] = br;
    wheels[1] = bl;
    wheels[2] = fr;
    wheels[3] = fl;
    
    br.speed_m.setInverted(true);
    fr.speed_m.setInverted(true);
  }

  public double[][] trig (double x1, double y1, double x2) {
    double sine = Constants.side_over_radius;

    double a = x1 - x2 * sine;
    double b = x1 + x2 * sine;
    double c = y1 - x2 * sine;
    double d = y1 + x2 * sine;

    double[] Speeds = {
      Math.sqrt ((b * b) + (d * d)),
      Math.sqrt ((b * b) + (c * c)),
      Math.sqrt ((a * a) + (d * d)),
      Math.sqrt ((a * a) + (c * c))};

    double[] Angles = {
      Math.toDegrees(Math.atan2 (b, d)),
      Math.toDegrees(Math.atan2 (b, c)),
      Math.toDegrees(Math.atan2 (a, d)),
      Math.toDegrees(Math.atan2 (a, c))};

    double[][] dir = {Speeds, Angles};
    return dir;
  }

  /* motor.getSelectedSensorPosition() + 
  RobotContainer.angleDistance2(angles[i], currentAngles[i])*Constants.pos_units_per_degree * 
  (RobotContainer.shouldTurnLeft(currentAngles[i], angles[i]) ? 1:-1)))); */
  public void drive (double x1, double y1, double x2) {
    double[][] dir = trig(x1, y1, x2);
    double[] currAngVal = getValues(4, 8);
    double[] currAng = getCurrAng(); 
    for (short i = 0; i < 4; i++) {
      if(RobotContainer.angleDistance2(dir[1][i], currAng[i]) > 90){
        thrustCoefficients[i] = -1;
        dir[1][i] = (dir[1][i]+180)%360;
        wheels[i].drive(dir[0][i], dir[1][i], thrustCoefficients[i]);
      } else {
        thrustCoefficients[i] = 1;
        wheels[i].drive(dir[0][i], ((currAngVal[i] + 
        RobotContainer.angleDistance2(dir[1][i], currAng[i])*Constants.units_per_degree * 
        (RobotContainer.shouldTurnLeft(currAng[i], dir[1][i]) ? 1:-1))), thrustCoefficients[i]);
      }
    }
  }

  public void drive_fo (double x1, double y1, double x2) {
    double foAngle = getFOAngle();
    double temp = y1 * Math.cos(foAngle) + x1 * Math.sin(foAngle);
    x1 = -y1 * Math.sin(foAngle) + x1 * Math.cos(foAngle);
    y1 = temp;
    drive(x1,y1,x2);
  }

  public void stop() {
    for(Wheel el:wheels) {el.drive(0, 0, 0);}
  }

  public double[] getValues(int first, int last ) {
    double[] positions = new double[8];
    for (;first < last; first++) {
      if (first<4) {
        positions[first] = wheels[first].speed_m.getSelectedSensorPosition()*Constants.distancePP;
      } else {
        positions[first] = wheels[first-4].angle_m.getSelectedSensorPosition()/Constants.units_per_degree;
      }
    }
    return positions;
  }

  public double[] getCurrAng() {
    double[] currAng = getValues(4, 8);
    for(int i = 0; i<4; i++){
      currAng[i] = RobotContainer.floorMod(currAng[i]/Constants.units_per_degree, 360);
    }
    return currAng;
  }

  public double getFOAngle(){
    double angle = ahrs.getAngle();
    if (angle<=0) angle += 360;
    return (360-angle)%360;
  }

  public void setPID() {
    for (Wheel el:wheels) {
      el.kpDir = kpDirEntry.getDouble(el.kpDir);
      el.kiDir = kiDirEntry.getDouble(el.kiDir);
      el.kdDir = kdDirEntry.getDouble(el.kdDir);
  
      el.kpTh = kpThEntry.getDouble(el.kpTh);
      el.kiTh = kiThEntry.getDouble(el.kiTh);
      el.kdTh = kdThEntry.getDouble(el.kdTh);
      el.kfTh = kfThEntry.getDouble(el.kfTh);
  
      el.angle_m.config_kP(el.slotIdx, el.kpDir);
      el.angle_m.config_kI(el.slotIdx, el.kiDir);
      el.angle_m.config_kD(el.slotIdx, el.kdDir);
  
      el.speed_m.config_kP(el.slotIdx, el.kpTh);
      el.speed_m.config_kI(el.slotIdx, el.kiTh);
      el.speed_m.config_kD(el.slotIdx, el.kdTh);
      el.angle_m.config_kF(el.slotIdx, el.kfTh);
    }
  }
}
