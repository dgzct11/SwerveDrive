package frc.robot.functional.trajectory;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class Segment {
    public double[] startPoint, endPoint;
    public double startTime, endTime, length, midTime;
    public void setModeToRamp(){
        midTime = -1;
    }
    public void setMidTime(double num){
        midTime = num;
    }
    public void setModeToConstant(){
        midTime = -2;
    }
    public void setModeToBreak(){
        midTime = -3;
    }
    public Position getPosition(double distance){
        return new Position(0,0,0);
    }
  
}