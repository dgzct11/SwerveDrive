// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public void drive(double strafeAngle,double speed, double rotateAmount){
    //if stick is really far to the left, set each motor tangential to a circle with the center at the robots center.
    if(Math.abs(rotate)>Constants.spin_threshold){
      double angle = RobotContainer.to360(Math.toDegrees(Math.atan2(Constants.left_right_wheel_distance/2, Constants.front_back_wheel_distance/2)));
      leftFrontDirection.setSelectedSensorPosition(angle*Constants.pos_units_per_degree);
    leftBackDirection.setSelectedSensorPosition(-angle*Constants.pos_units_per_degree);
    rightFrontDirection.setSelectedSensorPosition(-angle*Constants.pos_units_per_degree);
    rightBackDirection.setSelectedSensorPosition(angle*Constants.pos_units_per_degree);
    return;
    }

    currentStrafeAngle = strafeAngle;
    double circleRadius = 1/rotateAmount*Constants.rotation_dampener;
    
    double[] angles = calcAngles(circleRadius);

    leftFrontDirection.setSelectedSensorPosition((angles[0])*Constants.pos_units_per_degree);
    leftBackDirection.setSelectedSensorPosition(angles[1]*Constants.pos_units_per_degree);
    rightFrontDirection.setSelectedSensorPosition(angles[2]*Constants.pos_units_per_degree);
    rightBackDirection.setSelectedSensorPosition(angles[3]*Constants.pos_units_per_degree);
    
    
  }

  public double[] calcAngles(double circleRadius){
    double[] result = new double[4];
    Line robotCenterLinePerpendicular = new Line(currentStrafeAngle+90,Constants.center);
    double[] circleCenter = {Math.cos(Math.toRadians(currentStrafeAngle+90))*circleRadius,Math.sin(Math.toRadians(currentStrafeAngle+90))*circleRadius};
    Line leftFront = new Line(currentStrafeAngle,Constants.leftFrontCenter);
    Line leftBack = new Line(currentStrafeAngle, Constants.leftBackCenter);
    Line rightFront = new Line(currentStrafeAngle,Constants.rightFrontCenter);
    Line rightBack = new Line(currentStrafeAngle, Constants.rightBackCenter);

    double[] leftFrontP = robotCenterLinePerpendicular.getIntersection(leftFront);
    double[] leftBackP = robotCenterLinePerpendicular.getIntersection(leftBack);
    double[] rightFrontP = robotCenterLinePerpendicular.getIntersection(rightFront);
    double[] rightBackP = robotCenterLinePerpendicular.getIntersection(rightBack);
  // result's order is leftF, leftB, rightF, rightB
    result[0] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.leftFrontCenter, leftFrontP), RobotContainer.distance(circleCenter, leftFrontP))))*(circleRadius>0 ? -1:1);
    result[1] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.leftBackCenter, leftBackP), RobotContainer.distance(circleCenter, leftBackP))))*(circleRadius>0 ? 1:-1);
    result[2] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.rightFrontCenter, rightFrontP), RobotContainer.distance(circleCenter, rightFrontP))))*(circleRadius>0 ? -1:1);
    result[3] =currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.rightBackCenter, rightBackP), RobotContainer.distance(circleCenter, rightBackP))))*(circleRadius>0 ? 1:-1);
    return result; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
