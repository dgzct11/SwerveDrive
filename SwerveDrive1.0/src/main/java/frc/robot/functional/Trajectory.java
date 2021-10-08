package frc.robot.functional;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import java.util.ArrayList;

import frc.robot.RobotContainer;
/** Add your docs here. */
public class Trajectory {
    double[][] points;
    double[] distances;
   
    ArrayList<Segment> segments;
    double maxVelocity;
    double acceleration;
    

    double totalDistance = 0;
    
    int currentIndex = 0;
    
    public double timeToMax = 0;
    public double timeToBreak = 0;
    public double totalTime = 0;
    public double distanceToAccelerate;
    public double distanceToBreak;
    public Trajectory(double[][] points, double[] distances, double a, double mv){
        this.points = points;
        this.distances = distances;
        acceleration = a;
        maxVelocity = mv;
        initializeSegments();
        getTotalDistance();
        /* timetomax = maxV/a 
        distance to accelerate: maxV^2 = 2ax
        */
        timeToMax = maxVelocity/acceleration;
        timeToBreak = (maxVelocity/acceleration);
        distanceToAccelerate = (maxVelocity*maxVelocity/(2*acceleration));
        distanceToBreak = (maxVelocity*maxVelocity/(2*acceleration));
        totalTime = (totalDistance - distanceToAccelerate - distanceToBreak )/maxVelocity + timeToMax + timeToBreak;
        
    }
    public void setMaxAV(double a, double v){
        acceleration = a;
        maxVelocity = v;
        
    }
    public Position getEndPoint(){
        Segment seg = segments.get(segments.size()-1);
            return new Position(seg.endPoint, RobotContainer.angleFromSlope(seg.startPoint, seg.endPoint));
    }
    public Position getPosition(double time){
    
        /*
       if time<acceltime:
        use x = x + vt + a/2t^2
        if time>breakingTime:

        else:
        */
        double distance;
        if(time>totalTime){
            Segment seg = segments.get(segments.size()-1);
            return new Position(seg.endPoint, RobotContainer.angleFromSlope(seg.startPoint, seg.endPoint));
        }
        if(time <= timeToMax){
             distance = acceleration/2*time*time;
            
           
        }
        else if(time >= totalTime-timeToBreak){
            distance = totalDistance-distanceToBreak + maxVelocity*(time-(totalTime-timeToBreak)) -acceleration/2*Math.pow((time-(totalTime-timeToBreak)),2);
        }
        else{
            distance = distanceToAccelerate + maxVelocity*(time-timeToMax);
        }
        if(distance>totalDistance){
            Segment seg = segments.get(segments.size()-1);
            return new Position(seg.endPoint, RobotContainer.angleFromSlope(seg.startPoint, seg.endPoint));
        }
        double currentDistance = 0;
        int index = 0;
        while(index<segments.size()-1 && currentDistance +segments.get(index).length <= distance){
            currentDistance += segments.get(index).length;   
            index ++;
        }
       
        
        return segments.get(index).getPosition(distance - currentDistance );

        //turn currentDistance into position
       
    }
    
    public String toString(){
        String result = "";
        for(Segment segment: segments)
            result += segment.toString();
        return result;
    }
    public void getTotalDistance(){
        for(Segment seg: segments){
           totalDistance += seg.length;
        }
      
    }
    public void initializeSegments(){
        segments = new ArrayList<Segment>();
        double[] startPoint = points[0];

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
            segments.add(line);
            segments.add(circle);
            startPoint = circleEnd;
        }
        segments.add(new Line(startPoint, points[points.length-1]));



    }

}