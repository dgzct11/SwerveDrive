// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.functional.Position;
import frc.robot.functional.Circle;
public class Odometry extends SubsystemBase {
  /** Creates a new Odometry. */
  public Odometry() {
    
  }
  public  Position currentPosition = new Position(0,0,0);
  DriveTrain driveTrain;
  double[] previousPositionsThrust = new double[4];
  double[] previousPositionsDirectional = new double[4];
  public Odometry(DriveTrain dt) {
    driveTrain = dt;
    previousPositionsDirectional = driveTrain.getDirectionalPositions();
    previousPositionsThrust = driveTrain.getThrustPositions();
  }
  public void updatePosition(){
    double[] currentPositionsThrust = driveTrain.getThrustPositions();
    double[] currentPositionDirectional = driveTrain.getDirectionalPositions();
    if(driveTrain.spinning){
      double distanceChange = 0;
      for(int i = 0; i<4; i++)
        distanceChange += currentPositionsThrust[i] - previousPositionsThrust[i];
      distanceChange /= 4;
      double angleChange = Math.toDegrees(distanceChange/Constants.distance_wheel_center);
      currentPosition.addAngle(angleChange);
    }
    else if (Double.isInfinite(driveTrain.currentCircleRadius)){
      double distanceChange = 0;
      for(int i = 0; i<4; i++)
        distanceChange += currentPositionsThrust[i] - previousPositionsThrust[i];
      distanceChange /= 4;
      double dx = Math.cos(Math.toRadians(driveTrain.currentStrafeAngle))*distanceChange;
      double dy = Math.sin(Math.toDegrees(driveTrain.currentStrafeAngle))*distanceChange;
      currentPosition.add(dx, dy);
    }
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
  }
}
