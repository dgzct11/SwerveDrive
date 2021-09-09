package frc.robot.functional;
import frc.robot.RobotContainer;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class Line extends Segment{
    public double slope;
   
   
    public Line(double[] start, double[] end){
        startPoint = start;
        endPoint = end;
        slope = (start[1] - end[1]) / (start[0]-end[0]);
        length = RobotContainer.distance(start, end);

    }
    public Line(double[] end, double s){
        endPoint = end;
        slope = s;
    }
    public Line(double angle, double[] end){
        endPoint = end;
        slope = Math.tan(Math.toRadians(angle));
    }

    public double[] getIntersection(Line line){
        
        double[] result = new double[2];
        if(Math.abs(line.slope) == Double.POSITIVE_INFINITY){
            result[0] = line.endPoint[0];
            result[1] =  slope * (result[0] - endPoint[0]) + endPoint[1];
            return result;
        }
        else if(Math.abs(slope) == Double.POSITIVE_INFINITY){
            result[0] =  endPoint[0];
            result[1] = line.slope * (result[0] - line.endPoint[0]) + line.endPoint[1];
            return result;
        }

        result[0] = (-line.slope * line.endPoint[0] + line.endPoint[1] - endPoint[1] + slope * endPoint[0])/(slope - line.slope);
        result[1] = slope * (result[0] - endPoint[0]) + endPoint[1];
        return result;
    }

    public double[] getMidPoint(){
        double[] result = new double[2];
        result[0] = (endPoint[0] - startPoint[0])/2 + startPoint[0]; 
        result[1] = (endPoint[1] - startPoint[1])/2 + startPoint[1];
        return result; 
    }
    public Position getPosition(double distance){
        return new Position(distance/length * (endPoint[0]-startPoint[0]) + startPoint[0], 
        distance/length * (endPoint[1]-startPoint[1]) + startPoint[1],
         RobotContainer.angleFromSlope(startPoint, endPoint));
    }
    public String toString(){
        try{
        return String.format("Line: \n\tslope: %f\n\tstart: %f %f\n\tend: %f %f\n\tlength: %f\n", slope,
        startPoint[0], startPoint[1], endPoint[0], endPoint[1], length);
        } catch(NullPointerException e){
            return String.format("\nLine: \n\tslope: %f\n\tpoint: %f %f\n", slope,
            endPoint[0], endPoint[1],length);
        }
    }
}
