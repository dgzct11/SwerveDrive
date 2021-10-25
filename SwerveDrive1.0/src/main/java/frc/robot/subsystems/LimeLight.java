// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.functional.ThreeDProjectile;
import frc.robot.functional.trajectory.Position;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  //horizontal angle offset to target
  NetworkTableEntry tx = table.getEntry("tx");
  //vertical angle offset to target
  NetworkTableEntry ty = table.getEntry("ty");

  //how much of the cameras area is taken up by the object
  //used for distance estimation. 
  NetworkTableEntry ta = table.getEntry("ta");

  NetworkTableEntry tv = table.getEntry("tv");
  
  double x;
  double y;
  boolean objectInView;
  double area;
  double[] previousBallPosition; 
  double previousBallTime;
  Odometry odometry;

  public LimeLight(Odometry od) {
    odometry = od;
  }

  public void updateValues(){
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    objectInView = (tv.getDouble(0.0) == 1);
    previousBallPosition = getObjectPosition(Constants.ball_area, odometry.currentPosition);
    previousBallTime = System.currentTimeMillis()/1000;
  }
  public double getDistance(double objectHeight){
    double heightDifference = objectHeight-Constants.limeLightHeight;
    return heightDifference/Math.tan(Math.toRadians(getHorizontalAngleDiff())); 
  }
  public double getDistanceFromArea(double objectArea){
    //returns distance to plane.
    double totalCameraArea = objectArea/(area/100);
    double horizontalWidthOfFrame = Math.sqrt(totalCameraArea/0.733789999804);
    return horizontalWidthOfFrame/2/0.509525449494;
  }
  public double getHorizontalAngleDiff(){
    return x;
    //positive x is right, negative x is left
  }
  public double[] getObjectPosition(double objectArea, Position currentPosition){
  
    double pointDistance = getDistanceFromArea(objectArea)/Math.cos(Math.toRadians(Math.abs(getHorizontalAngleDiff())));
    double angle = NavXGyro.getAngle() - getHorizontalAngleDiff();
    double height = Math.tan(Math.toRadians(getVerticalAngleDiff())) + Constants.limeLightHeight;
    double[] result = {
      -Math.sin(Math.toRadians(angle))*pointDistance + currentPosition.x,
      Math.cos(Math.toRadians(angle)) * pointDistance + currentPosition.y,
      height
    };
    return result;
  }
  public double[] trackBall(){
    double[] currentPosition = getObjectPosition(Constants.ball_area, odometry.currentPosition);
    double deltaTime = System.currentTimeMillis()/1000 - previousBallTime;
    double[] velocities = {
      (currentPosition[0] - previousBallPosition[0])/deltaTime,
      (currentPosition[1] - previousBallPosition[1])/deltaTime,
      (currentPosition[2] - previousBallPosition[2])/deltaTime
    };
    double[] landingZoneTime = ThreeDProjectile.getLandingPosTime(velocities, currentPosition);
    previousBallPosition = currentPosition;
    return landingZoneTime;
  }

  public double getVerticalAngleDiff(){
    return y;
  }
  public boolean inView(){
    return objectInView;
  }
  public void outputToDash(){
 
    SmartDashboard.putNumber("Lime Area", area);
    SmartDashboard.putNumber("Ball Distance", getDistanceFromArea(0.0316));
   
    double[] landingZoneTime = trackBall();
    SmartDashboard.putNumber("Ball X", landingZoneTime[0]);
    SmartDashboard.putNumber("Ball Y", landingZoneTime[1]);
    SmartDashboard.putNumber("Ball Time", landingZoneTime[2]);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateValues();
    outputToDash();
  }
}
