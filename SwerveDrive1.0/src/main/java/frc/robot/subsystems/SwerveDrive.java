// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.functional.Wheel;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



/** An example command that uses an example subsystem. */
public class SwerveDrive extends SubsystemBase{

  private Wheel[] wheels = new Wheel[4];
  public int[] thrustCoefficients = {1,1,1,1};
  public static AHRS ahrs = new AHRS(Constants.mxp_port);

  public SwerveDrive (Wheel br, Wheel bl, Wheel fr, Wheel fl) {
    wheels[0] = br;
    wheels[1] = bl;
    wheels[2] = fr;
    wheels[3] = fl;
    br.speed_m.setInverted(true);
    fr.speed_m.setInverted(true);
    ahrs.reset();
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

    double brAngle = Math.toDegrees(Math.atan2 (b, d));
    double blAngle = Math.toDegrees(Math.atan2 (b, c));
    double frAngle = Math.toDegrees(Math.atan2 (a, d));
    double flAngle = Math.toDegrees(Math.atan2 (a, c));

    double[][] dir = {{brSpeed, blSpeed, frSpeed, flSpeed},{brAngle, blAngle, frAngle, flAngle}};

    double maxS = 0; double maxD = 0;
    for (short i = 0; i < 4; i++) {
      if (dir[0][i] > maxS) {maxS = dir[0][i];};
      if (dir[1][i] > maxD) {maxD = dir[1][i];};
      dir[0][i] /= maxS; dir[1][i] /= maxD;
    }
    return dir;
  }

  public void drive (double x1, double y1, double x2) {
    double[][] dir = trig(x1, y1, x2);
    for (short i = 0; i < 4; i++) {
      wheels[i].drive(dir[0][i], dir[1][i]);
    }
  }

  public void drive_fo (double x1, double y1, double x2) {
    double foAngle = getFOAngle();
    double temp = y1 * Math.cos(foAngle) + x1 * Math.sin(foAngle);
    x1 = -y1 * Math.sin(foAngle) + x1 * Math.cos(foAngle);
    y1 = temp;
    double[][] dir = trig(x1, y1, x2);
    for (short i = 0; i < 4; i++) {
      wheels[i].drive(dir[0][i], dir[1][i]);
    }
  }

  public void driveDirectionalAngles(double[] angles) {
    double[] currentAngles = getValues(4, 8);
    for(int i = 0; i<4; i++) {
      wheels[i].drive(0, (currentAngles[i] + angles[i]));
    }
  }

  public void stop() {
    for(Wheel el:wheels) {el.drive(0, 0);}
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

  public double getFOAngle(){
    return (360-(ahrs.getAngle()+360));
  }

  public void setPID() {
    for (Wheel el:wheels) {
      el.setPID();
    }
  }
}
