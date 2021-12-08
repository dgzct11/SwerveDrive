// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.functional.trajectory.Position;

public class Odometry extends SubsystemBase {
 
  public  Position currentPosition = new Position(0,0,0);
  DriveTrain driveTrain;
  //the position of every Talon thrust
  double[] previousPositionsThrust = new double[4];
  //the position of every Talon directional
  double[] previousAngles = new double[4];
  double previousTime = 0;

  double[][] currentPoses = new double[4][3];
 
  double[] previousTransVector = new double[2];
  public Odometry(){

  }

  public void setDriveTrain(DriveTrain dt){
    driveTrain = dt;
    previousAngles =  driveTrain.getAngles(); 
    previousPositionsThrust = driveTrain.getThrustPositions();
  }

  public Odometry(DriveTrain dt) {
    driveTrain = dt;
   previousPositionsThrust = driveTrain.getThrustPositions();
   previousAngles = driveTrain.getAngles();
  }

  public void reset(){
    currentPosition = new Position(0, 0, 0);
  }
  public Position getPosition(){
    return currentPosition;
  }
  public double getAngle(){
    return currentPosition.angle;
  }
  public double getSlip(){
    double maxAcceleration = 1;
    double[] angles = driveTrain.getAngles();
    int[] thrustDirections = driveTrain.getThrustCoefficients();
    double[] deltaPositions = driveTrain.getThrustPositions();
    for(int i = 0; i<4; i++){
      deltaPositions[i] -=  previousPositionsThrust[i];
      if(thrustDirections[i] == -1){
        angles[i] = (angles[i] + 180)%360;
        deltaPositions[i] *= -1;
      }
    }
    previousPositionsThrust = driveTrain.getThrustPositions();
    double[] strafeVector = new double[2];
    //vector for each wheel rpresenting x y movement
    double[][] displacementVectors = new double[4][2];
    
 
    double currentAngle = NavXGyro.getAngle();
    for(int i = 0; i<4; i++){

      //each displacemenet vector is vector for each wheel
      displacementVectors[i][0] = -Math.sin(Math.toRadians(angles[i] + currentAngle)) * Math.abs(deltaPositions[i]);
      displacementVectors[i][1] =  Math.cos(Math.toRadians(angles[i] + currentAngle)) * Math.abs(deltaPositions[i]);

      //adds x and y to avarage vector
      // avarage vector represents movement of center, since the sum of all rotation vectors should be zero
      strafeVector[0] += displacementVectors[i][0]/4/Constants.pos_units_per_meter;
      strafeVector[1] += displacementVectors[i][1]/4/Constants.pos_units_per_meter;
    }
    double dt = previousTime - System.currentTimeMillis()/1000.;
    double[] acceleration = {
      (strafeVector[0]- previousTransVector[0])/dt,
      (strafeVector[1]- previousTransVector[1])/dt,
    };
    double mag = RobotContainer.magnitutde(acceleration);
    if(mag > maxAcceleration){
      acceleration[0] *= (maxAcceleration/mag);
      acceleration[1] *= (maxAcceleration/mag);
    }
    double[] actualStrafe = {
      previousTransVector[0] + acceleration[0]*dt,
      previousTransVector[1] + acceleration[1]*dt
    };
  
    double distance = RobotContainer.distance(strafeVector, actualStrafe);
    previousTransVector = strafeVector;
    return distance;

  }
  public void updateIndividualPose(){
    double[] angles = driveTrain.getAngles();
    int[] thrustDirections = driveTrain.getThrustCoefficients();
    double[] deltaPositions = driveTrain.getThrustPositions();
    double heading = NavXGyro.getAngle();
    
    for(int i = 0; i<4; i++){
      deltaPositions[i] -=  previousPositionsThrust[i];
      if(thrustDirections[i] == -1){
        angles[i] = (angles[i] + 180)%360;
        deltaPositions[i] *= -1;
      }
      angles[i] = (angles[i] + heading)%360;
      double angleDiff = RobotContainer.angleDistance2(currentPoses[i][2], angles[i]);
      double radius = deltaPositions[i] * 360 / ((double)(angleDiff*2*Math.PI));
      double[] circleCenter = {
        currentPoses[i][0]-radius*Math.sin(Math.toRadians(currentPoses[i][2] - 90)),
        currentPoses[i][1] + radius*Math.cos(Math.toRadians(currentPoses[i][2] - 90))
       };
      double[] newPose = {
        -radius*Math.sin(Math.toRadians(angles[i])) + circleCenter[0],
        radius*Math.cos(Math.toRadians(angles[i])) + circleCenter[1]
      };
      currentPoses[i][0] = newPose[0];
      currentPoses[i][1] = newPose[1];
      currentPoses[i][2] = angles[i];

    }
    previousPositionsThrust = driveTrain.getThrustPositions();

    
  }


  public void updatePosition(){
    //get current Vector for center 
    //avarage of all vectors removes rotational component

    //get angles of wheels
    double[] angles = driveTrain.getAngles();
    
    /*
    for(int i = 0; i<4; i++){
      angles[i] = (angles[i] + previousAngles[i])/2;
    }*/
    
    int[] thrustDirections = driveTrain.getThrustCoefficients();

    //get how far each wheel traveled since last iteration
    double[] deltaPositions = driveTrain.getThrustPositions();
    //subtracts previous position from total position to get change in position
    for(int i = 0; i<4; i++){
      deltaPositions[i] -=  previousPositionsThrust[i];
      if(thrustDirections[i] == -1){
        angles[i] = (angles[i] + 180)%360;
        deltaPositions[i] *= -1;
      }
    }
    previousPositionsThrust = driveTrain.getThrustPositions();

    double[] strafeVector = new double[2];
    //vector for each wheel rpresenting x y movement
    double[][] displacementVectors = new double[4][2];
    
 
    double currentAngle = NavXGyro.getAngle();
    for(int i = 0; i<4; i++){

      //each displacemenet vector is vector for each wheel
      displacementVectors[i][0] = -Math.sin(Math.toRadians(angles[i] + currentAngle)) * Math.abs(deltaPositions[i]);
      displacementVectors[i][1] =  Math.cos(Math.toRadians(angles[i] + currentAngle)) * Math.abs(deltaPositions[i]);

      //adds x and y to avarage vector
      // avarage vector represents movement of center, since the sum of all rotation vectors should be zero
      strafeVector[0] += displacementVectors[i][0]/4;
      strafeVector[1] += displacementVectors[i][1]/4;
    }

    

    currentPosition.add(strafeVector[0]/Constants.pos_units_per_meter, strafeVector[1]/Constants.pos_units_per_meter);
   
    
  
    
    //get current rotational vector

  }

  public double[] getAveragePose(){
    double[] result = new double[2];
    for(int i = 0; i<4; i++){
      result[0] += currentPoses[i][0];
      result[1] += currentPoses[i][1];
    
    }
    result[0] /= 4;
    result[1] /= 4;
    return result;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePosition();
    updateIndividualPose();
    double[] pose = getAveragePose();
   
    SmartDashboard.putNumber("Od X", currentPosition.x);
    SmartDashboard.putNumber("Od Y", currentPosition.y);

    SmartDashboard.putNumber("New Od X", pose[0]);
    SmartDashboard.putNumber("New Od Y", pose[1]);
    
    SmartDashboard.putNumber("Od Slip", getSlip());
  }
}
