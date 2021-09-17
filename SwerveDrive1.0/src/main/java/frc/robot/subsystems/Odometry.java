// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.functional.Position;
import frc.robot.functional.Circle;
public class Odometry extends SubsystemBase {
 
  public  Position currentPosition = new Position(0,0,0);
  DriveTrain driveTrain;
  //the position of every Talon thrust
  double[] previousPositionsThrust = new double[4];
  //the position of every Talon directional
  double[] previousPositionsDirectional = new double[4];
  public Odometry(){

  }
  public void setDriveTrain(DriveTrain dt){
    driveTrain = dt;
    previousPositionsDirectional = driveTrain.getDirectionalPositions();
    previousPositionsThrust = driveTrain.getThrustPositions();
  }
  public Odometry(DriveTrain dt) {
    driveTrain = dt;
    previousPositionsDirectional = driveTrain.getDirectionalPositions();
    previousPositionsThrust = driveTrain.getThrustPositions();
  }
  public void reset(){
    currentPosition = new Position(0, 0, 0);
  }
  public Position getPosition(){
    return currentPosition;
  }
  public void updatePosition(){
    //get current Vector for center 
    //avarage of all vectors removes rotational component
    double[] angles = driveTrain.getAngles();
    double[] deltaPositions = driveTrain.getThrustPositions();
    for(int i = 0; i<4; i++)
      deltaPositions[i] -= previousPositionsThrust[i];
    
    double[] avgVector = new double[2];
    double[][] displacementVectors = new double[4][2];
    double avgRotationMag = 0;
    for(int i = 0; i<4; i++){
      displacementVectors[i][0] = Math.sin(Math.toRadians(angles[i]))*Math.abs(deltaPositions[i]);
      displacementVectors[i][1] =  Math.cos(Math.toRadians(angles[i]))*Math.abs(deltaPositions[i]);
      avgVector[0] += displacementVectors[i][0]/4;
      avgVector[1] += displacementVectors[i][1]/4;
      displacementVectors[i][0] -= avgVector[0];
      displacementVectors[i][1] -= avgVector[1];
      avgRotationMag += RobotContainer.magnitutde(displacementVectors[i])/4;
    }
    double angleDiff = Math.toDegrees(avgRotationMag/Constants.distance_wheel_center);
    
    currentPosition.add(avgVector[0], avgVector[1]);
    currentPosition.addAngle(angleDiff);
    
  
    
    //get current rotational vector

  }

  public void updateArcPosition(){
    double[] currentPositionsThrust = driveTrain.getThrustPositions();
    //double[] currentPositionDirectional = driveTrain.getDirectionalPositions();
    
    //if the robot is spinning on its axis the only value that will change is the angle
    if(driveTrain.spinning){
      double distanceChange = 0;
      for(int i = 0; i<4; i++)
        distanceChange += currentPositionsThrust[i] - previousPositionsThrust[i];
      distanceChange /= 4;
      double angleChange = Math.toDegrees(distanceChange/Constants.distance_wheel_center);
      currentPosition.addAngle(angleChange);
    }

    //if the current turning circle is infinitely large, the robot is strafing
    else if (Double.isInfinite(driveTrain.currentCircleRadius)){
      double distanceChange = 0;
      for(int i = 0; i<4; i++)
        distanceChange += currentPositionsThrust[i] - previousPositionsThrust[i];
      distanceChange /= 4;
      double dx = Math.cos(Math.toRadians(driveTrain.currentStrafeAngle))*distanceChange;
      double dy = Math.sin(Math.toDegrees(driveTrain.currentStrafeAngle))*distanceChange;
      currentPosition.add(dx, dy);
    }

    //if the turning circle is not infinitely small, 
    //the angle and position will change according to the radius of the turning circle
    else{
      double angleChange = 0;
      for(int i =0; i<4; i++)
        angleChange += Math.toDegrees((currentPositionsThrust[i] - previousPositionsThrust[i])/driveTrain.currentCircleRadius);
      angleChange /= 4;
      
      Circle circle = new Circle(driveTrain.currentCircleCenter, Math.abs(driveTrain.currentCircleRadius), Constants.center);
      Position newPos = circle.getPositionFromAngle(angleChange);
      currentPosition.add(newPos.x, newPos.y);
      currentPosition.addAngle(angleChange);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePosition();
  }
}
