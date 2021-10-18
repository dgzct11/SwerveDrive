// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

/** Add your docs here. */
public class ThreeDProjectile {
    public static double[] getLandingPosTime(double[] velocities, double[] positions){
        
        double roots[] = quadratic(-4.9, velocities[2], positions[2]);
        double flightTime = Math.max(roots[0], roots[1]);
        double[] result = {
            positions[0] + velocities[0] * flightTime,
            positions[1] + velocities[1] * flightTime
        };
        return result;
        //returns [x,y,t]
    }
    public static double[] quadratic(double a, double b, double c){
        double[] result = {
            (-b + Math.sqrt(Math.pow(b, 2) - 4*a*c))/(2*a),
            (-b - Math.sqrt(Math.pow(b, 2) - 4*a*c))/(2*a)
        };
        return result;
    }
    
}
