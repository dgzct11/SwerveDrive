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

  public LimeLight() {

  }

  public void updateValues(){
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    objectInView = (tv.getDouble(0.0) == 1);


  }
  public double getDistance(double objectHeight){
    double heightDifference = objectHeight-Constants.limeLightHeight;
    return heightDifference/Math.tan(Math.toRadians(getHorizontalAngleDiff())); 
  }
  public double getDistanceFromArea(double objectArea){
    double totalCameraArea = objectArea/(area/100);
    double horizontalWidthOfFrame = Math.sqrt(totalCameraArea/0.733789999804);
    return horizontalWidthOfFrame/2/0.509525449494;
  }
  public double getHorizontalAngleDiff(){
    return -x;
  }
  public double getVerticalAngleDiff(){
    return y;
  }
  public boolean inView(){
    return objectInView;
  }
  public void outputToDash(){
    SmartDashboard.putNumber("Lime X", x);
    SmartDashboard.putNumber("Lime y", y);
    SmartDashboard.putNumber("Lime Area", area);
    SmartDashboard.putNumber("Distance", getDistanceFromArea(0.0316));
    SmartDashboard.putBoolean("Lime Tracking", objectInView);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateValues();
    outputToDash();
  }
}
