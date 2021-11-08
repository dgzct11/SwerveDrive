// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional.trajectory;

/** Add your docs here. */
public class KinematicSegment{
    public double[] start;
    public double[] end;
    double totalTime;
    double acceleration;
    double distance;
    public KinematicSegment(double[] s, double[] e){
        start = s;
        end = e;
        // vf^2 = v0^2 + 2a(x)
        // vf = v0 + at
        distance = e[0] - s[0];
        acceleration = ( e[1]*e[1] - s[1]*s[1] )/(2*distance);
        totalTime = (e[1] - s[1])/acceleration;
       
      
    }
    public double getDistance( double time) {
        //Xf = 0 + V0t +aT^2/2
        return start[1]*time + acceleration * time*time/2;
    }

}
