package frc.robot.functional.trajectory;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import java.util.ArrayList;

import frc.robot.RobotContainer;
/** Add your docs here. */
public class TrajectoryCircleLine extends Trajectory{
    double[][] points;
    double[] distances;

   
   
    public TrajectoryCircleLine(double[][] points, double[] distances, double[] angles, double a, double mv){
        super(a, mv, angles); 
        this.points = points;
        this.distances = distances;
       initializeSegments();
       getTotalDistance();
    }
  
    public void initializeSegments(){
        segments = new ArrayList<Segment>();
        double[] startPoint = points[0];
        int angleIndex = 0;
        for(int i = 0; i<points.length-2; i++){
            //establish points
            
            double[] cornerPoint = points[i+1];
            double[] nextCorner = points[i+2];
            double distance = RobotContainer.distance(startPoint, cornerPoint);
            double ratio = (distance - distances[i])/distance;
            double[] circleStart = { (cornerPoint[0]-startPoint[0])*ratio + startPoint[0], (cornerPoint[1]-startPoint[1])*ratio + startPoint[1] };

            double distance2 = RobotContainer.distance(nextCorner, cornerPoint);
            double ratio2 = (distances[i])/distance2;
            double[] circleEnd = { (nextCorner[0]-cornerPoint[0])*ratio2 + cornerPoint[0], (nextCorner[1]-cornerPoint[1])*ratio2 + cornerPoint[1] };
           
            //find circle equation
            double firstSlope = (startPoint[1]-cornerPoint[1])/(startPoint[0]-cornerPoint[0]);
            double firstPerpendicular = -1/firstSlope;
            double secondSlope = (nextCorner[1]-cornerPoint[1])/(nextCorner[0]-cornerPoint[0]);
            double secondPerpendicular = -1/secondSlope;

            Line line1 = new Line(circleStart , firstPerpendicular);
           
            //fix circle end;
            Line line2 = new Line(circleEnd,  secondPerpendicular);
           

            double[] center = line1.getIntersection(line2);
            double radius = RobotContainer.distance(center, circleStart);
           
            Circle circle = new Circle(center, radius, circleStart, circleEnd);
            Line line = new Line(startPoint, circleStart);
            line.angle = angles[angleIndex];
            angleIndex ++;
            circle.angle = angles[angleIndex];
            segments.add(line);
            segments.add(circle);
            startPoint = circleEnd;
        }
        Line line = new Line(startPoint, points[points.length-1]);
        line.angle = angles[angles.length-1];
        segments.add(line);



    }

}