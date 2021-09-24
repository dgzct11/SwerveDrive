// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public double getAngle(){
    return currentPosition.angle;
  }
  public void updatePosition(){
    //get current Vector for center 
    //avarage of all vectors removes rotational component

    //get angles of wheels
    double[] angles = driveTrain.getAngles();

    //get how far each wheel traveled since last iteration
    double[] deltaPositions = driveTrain.getThrustPositions();
    for(int i = 0; i<4; i++)
      deltaPositions[i] -= previousPositionsThrust[i];
    

    double[] avgVector = new double[2];
    double[][] displacementVectors = new double[4][2];
    double avgRotationMag = 0;

    for(int i = 0; i<4; i++){

      //each displacemenet vector is vector for each wheel
      displacementVectors[i][0] = -Math.sin(Math.toRadians(angles[i])) * Math.abs(deltaPositions[i]);
      displacementVectors[i][1] =  Math.cos(Math.toRadians(angles[i])) * Math.abs(deltaPositions[i]);

      //adds x and y to avarage vector
      // avarage vector represents movement of center
      avgVector[0] += displacementVectors[i][0]/4;
      avgVector[1] += displacementVectors[i][1]/4;
    }

    for(int i = 0; i<4; i++){
      //removes the average vector from each vector of the wheel
      //the rotation vectors remain
      displacementVectors[i][0] -= avgVector[0];
      displacementVectors[i][1] -= avgVector[1];
      avgRotationMag += RobotContainer.magnitutde(displacementVectors[i])/4;
    }
    double angleDiff = Math.toDegrees(avgRotationMag/Constants.pos_units_per_meter/Constants.distance_wheel_center);
    
    currentPosition.add(avgVector[0]/Constants.pos_units_per_meter, avgVector[1]/Constants.pos_units_per_meter);
    currentPosition.addAngle(angleDiff);
    
  
    previousPositionsThrust = driveTrain.getThrustPositions();
    //get current rotational vector

  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePosition();
    SmartDashboard.putNumber("Od Angle", currentPosition.angle);
    SmartDashboard.putNumber("Od X", currentPosition.x);
    SmartDashboard.putNumber("Od Y", currentPosition.y);
  }
}
