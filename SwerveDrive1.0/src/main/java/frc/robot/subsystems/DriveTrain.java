// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.functional.Line;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  public TalonSRX leftFrontDirection = new TalonSRX(Constants.left_front_direction_port);
  public TalonSRX leftFrontThrust = new TalonSRX(Constants.left_front_thrust_port);

  public TalonSRX rightFrontDirection = new TalonSRX(Constants.right_front_direction_port);
  public TalonSRX rightFrontThrust = new TalonSRX(Constants.right_front_thrust_port);

  public TalonSRX leftBackDirection = new TalonSRX(Constants.left_back_direction_port);
  public TalonSRX leftBackThrust = new TalonSRX(Constants.left_back_thrust_port);

  public TalonSRX rightBackDirection = new TalonSRX(Constants.right_back_direction_port);
  public TalonSRX rightBackThrust = new TalonSRX(Constants.right_back_thrust_port);

  //utility variables
  public double currentStrafeAngle = 0;
  public double currentCircleRadius = 0;
  public boolean spinning = false;
  public double[] currentCircleCenter = new double[2];
  public double[] currentWheelDistanceFromCircleCenter = new double[4];
  public DriveTrain() {
    //configure sensors for each motor controller
    leftFrontDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFrontThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftBackDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftBackThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightFrontDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightBackDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBackThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void drive(double strafeAngle,double speed, double circleRadius){
    //if stick is really far to the left, set each motor tangential to a circle with the center at the robots center.
    currentCircleRadius = circleRadius;
    currentStrafeAngle = strafeAngle;
    if(Math.abs(circleRadius)>Constants.spin_threshold){
      
        leftFrontDirection.setSelectedSensorPosition(Constants.spin_angle*Constants.pos_units_per_degree);
        leftBackDirection.setSelectedSensorPosition(-Constants.spin_angle*Constants.pos_units_per_degree);
        rightFrontDirection.setSelectedSensorPosition(-Constants.spin_angle*Constants.pos_units_per_degree);
        rightBackDirection.setSelectedSensorPosition(Constants.spin_angle*Constants.pos_units_per_degree);

        leftFrontThrust.set(ControlMode.PercentOutput, speed);
        leftBackThrust.set(ControlMode.PercentOutput, speed);
        rightFrontThrust.set(ControlMode.PercentOutput, speed);
        rightBackThrust.set(ControlMode.PercentOutput, speed);
    
    }
    
    else if(! (Math.abs(circleRadius) == Double.POSITIVE_INFINITY)){
        double[] angles_speeds = calcAngles(circleRadius);

        leftFrontDirection.setSelectedSensorPosition((angles_speeds[0])*Constants.pos_units_per_degree);
        leftBackDirection.setSelectedSensorPosition(angles_speeds[1]*Constants.pos_units_per_degree);
        rightFrontDirection.setSelectedSensorPosition(angles_speeds[2]*Constants.pos_units_per_degree);
        rightBackDirection.setSelectedSensorPosition(angles_speeds[3]*Constants.pos_units_per_degree);
      
        leftFrontThrust.set(ControlMode.PercentOutput, speed*angles_speeds[4]);
        leftBackThrust.set(ControlMode.PercentOutput, speed*angles_speeds[5]);
        rightFrontThrust.set(ControlMode.PercentOutput, speed*angles_speeds[6]);
        rightBackThrust.set(ControlMode.PercentOutput, speed*angles_speeds[7]);
      
    }
    else{
        leftFrontThrust.set(ControlMode.PercentOutput, speed);
        leftBackThrust.set(ControlMode.PercentOutput, speed);
        rightFrontThrust.set(ControlMode.PercentOutput, speed);
        rightBackThrust.set(ControlMode.PercentOutput, speed);
    }
    
  }

  public double[] calcAngles(double circleRadius){
    double[] result = new double[8];
    Line robotCenterLinePerpendicular = new Line(currentStrafeAngle+90,Constants.center);
    double[] circleCenter = {Math.cos(Math.toRadians(currentStrafeAngle+90))*circleRadius,Math.sin(Math.toRadians(currentStrafeAngle+90))*circleRadius};
    currentCircleCenter = circleCenter;
    Line leftFront = new Line(currentStrafeAngle,Constants.leftFrontCenter);
    Line leftBack = new Line(currentStrafeAngle, Constants.leftBackCenter);
    Line rightFront = new Line(currentStrafeAngle,Constants.rightFrontCenter);
    Line rightBack = new Line(currentStrafeAngle, Constants.rightBackCenter);

    double[] leftFrontP = robotCenterLinePerpendicular.getIntersection(leftFront);
    double[] leftBackP = robotCenterLinePerpendicular.getIntersection(leftBack);
    double[] rightFrontP = robotCenterLinePerpendicular.getIntersection(rightFront);
    double[] rightBackP = robotCenterLinePerpendicular.getIntersection(rightBack);

    double leftFrontRadius = RobotContainer.distance(leftFrontP, circleCenter);
    double leftBackRadius = RobotContainer.distance(leftBackP, circleCenter);
    double rightFrontRadius = RobotContainer.distance(rightFrontP, circleCenter);
    double rightBackRadius = RobotContainer.distance(rightBackP, circleCenter);
    currentWheelDistanceFromCircleCenter[0] = leftFrontRadius;
    currentWheelDistanceFromCircleCenter[1] = leftBackRadius;
    currentWheelDistanceFromCircleCenter[2] = rightFrontRadius;
    currentWheelDistanceFromCircleCenter[3] = rightBackRadius;
    double maxRadius  = Math.max(Math.max(Math.max(leftFrontRadius, leftBackRadius),rightFrontRadius),rightBackRadius);

  // result's order is leftF angle, leftB angle, rightF angle, rightB angle, 
  //leftF thrust, leftB thrust, rightF thrust, rightB thrust
    result[0] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.leftFrontCenter, leftFrontP), leftFrontRadius)))*(circleRadius>0 ? -1:1);
    result[1] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.leftBackCenter, leftBackP), leftBackRadius)))*(circleRadius>0 ? 1:-1);
    result[2] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.rightFrontCenter, rightFrontP), rightFrontRadius)))*(circleRadius>0 ? -1:1);
    result[3] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.rightBackCenter, rightBackP),rightBackRadius)))*(circleRadius>0 ? 1:-1);
    
    //thrust = radius/maxradius * speed
    result[4] = leftFrontRadius/maxRadius;
    result[5] = leftBackRadius/maxRadius;
    result[6] = rightFrontRadius/maxRadius;
    result[7] = rightBackRadius/maxRadius;
    return result; 
  }

  
  public double[] getThrustPositions(){
    double[] result = new double[4];
    result[0] = leftFrontThrust.getSelectedSensorPosition();
    result[1] = leftBackThrust.getSelectedSensorPosition();
    result[2] = rightFrontThrust.getSelectedSensorPosition();
    result[3] = rightBackThrust.getSelectedSensorPosition();
    return result;

  }
   
  public double[] getDirectionalPositions(){
    double[] result = new double[4];
    result[0] = leftFrontDirection.getSelectedSensorPosition();
    result[1] = leftBackDirection.getSelectedSensorPosition();
    result[2] = rightFrontDirection.getSelectedSensorPosition();
    result[3] = rightBackDirection.getSelectedSensorPosition();
    return result;

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
