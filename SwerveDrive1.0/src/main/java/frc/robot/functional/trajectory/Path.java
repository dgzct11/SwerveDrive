// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional.trajectory;

import java.util.ArrayList;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class Path {
    double[][] points;
    double[] distances;
    ArrayList<Segment> segments;
    public double totalDistance = 0;
    public int currentIndex = 0;
    public double[] angles;
    
   
   
    public Path(double[][] pts, double[] dist, double[] ang){
        
        angles = ang; 
        points = pts;
        distances = dist;
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
    
    public double getCurrentAngle(){
        return segments.get(currentIndex).angle;
    }
    
    public Position getEndPoint(){
        Segment seg = segments.get(segments.size()-1);
            return new Position(seg.endPoint, RobotContainer.angleFromSlope(seg.startPoint, seg.endPoint));
    }
    public Position getPosition(double distance){
        /*
       if time<acceltime:
        use x = x + vt + a/2t^2
        if time>breakingTime:

        else:
        */
       
        if(distance>totalDistance){
            Segment seg = segments.get(segments.size()-1);
            currentIndex = segments.size()-1;
            return new Position(seg.endPoint, RobotContainer.angleFromSlope(seg.startPoint, seg.endPoint));
        }
        double currentDistance = 0;
        int index = 0;
        while(index<segments.size()-1 && currentDistance +segments.get(index).length <= distance){
            currentDistance += segments.get(index).length;   
            index ++;
        }
        currentIndex = index;
        return segments.get(index).getPosition(distance - currentDistance );
        //turn currentDistance into position
    }
  
    public double getTotalDistance(){
        for(Segment seg: segments){
           totalDistance += seg.length;
        }
      return totalDistance;
    }

    public String toString(){
        String result = "";
        for(Segment segment: segments)
            result += segment.toString();
        return result;
    }
}
