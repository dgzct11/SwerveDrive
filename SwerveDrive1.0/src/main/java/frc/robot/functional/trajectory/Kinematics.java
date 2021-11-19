// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional.trajectory;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Kinematics {
    Path path;
    public double[][] velocities;
    public ArrayList<KinematicSegment> segments = new ArrayList<KinematicSegment>();
    public int currentIndex = 0;
    public double totalTime;
    public Kinematics(Path path, double[][] v){
        velocities = v;
        initializeSegments();
        getTotalTime();
    }
    public void initializeSegments(){
        for(int i = 0; i<velocities.length - 1; i++){
            segments.add(new KinematicSegment(velocities[i], velocities[i+1]));
        }
    }
    public double getTotalTime(){
        for(KinematicSegment segment:segments){
            totalTime += segment.totalTime;
        }
        return totalTime;
    }
    public double getDistance(double time){
        int index = 0;
        double timeSum = 0;
        double distanceSum = 0;
        if(time>totalTime) return path.totalDistance;
        while(segments.get(index).totalTime+timeSum<time){
            timeSum += segments.get(index).totalTime;
            distanceSum += segments.get(index).distance;
            index ++;
        }
      currentIndex = index;
        return segments.get(index).getDistance(time-timeSum)+distanceSum;
    }
}
