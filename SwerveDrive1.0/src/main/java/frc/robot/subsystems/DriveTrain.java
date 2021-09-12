// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
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
  //circle refers to circular path of rotation when turning
  public double currentStrafeAngle = 0;
  public double currentCircleRadius = 0;
  public boolean spinning = false;
  public double[] currentCircleCenter = new double[2];
  public double[] currentWheelDistanceFromCircleCenter = new double[4];





  public Odometry odometry;

  public DriveTrain(Odometry od) {
    //configure sensors for each motor controller to sensor in Falcon500
    leftFrontDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFrontThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftBackDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftBackThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightFrontDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightBackDirection.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBackThrust.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    odometry = od;
  }

  public void rotateDrive(double strafeAngle, double speed, double rotateSpeed){
    //positive rotate speed is left turn, negative rotate speed is right turn
    double strafeXComponent = Math.sin(Math.toRadians(strafeAngle))*speed;
    double strafeYComponent = Math.cos(Math.toRadians(strafeAngle))*speed;
    double rotationComponent = Constants.rotate_dampaner*rotateSpeed;
    double[] leftFrontVector = {strafeXComponent-rotationComponent, strafeYComponent-rotationComponent};
    double[] leftBackVector = {strafeXComponent - rotationComponent, strafeYComponent+rotationComponent};
    double[] rightFrontVector = {strafeXComponent+rotationComponent, strafeYComponent + rotationComponent};
    double[] rightBackVector = {strafeXComponent + rotationComponent, strafeYComponent - rotationComponent};
    double[] angles = {RobotContainer.stickTo360(leftFrontVector[0], leftFrontVector[1]),
                       RobotContainer.stickTo360(leftBackVector[0], leftBackVector[1]),
                       RobotContainer.stickTo360(rightFrontVector[0], rightFrontVector[1]),
                       RobotContainer.stickTo360(rightBackVector[0], rightBackVector[1])};
    double[] speeds = {RobotContainer.magnitutde(leftFrontVector),
                       RobotContainer.magnitutde(leftBackVector),
                       RobotContainer.magnitutde(rightFrontVector),
                       RobotContainer.magnitutde(rightBackVector)};
    setDirectionalAngles(angles);
    setThrustSpeeds(speeds);
    
  }

 

  public void arcDrive(double strafeAngle,double speed, double circleRadius){
   
    currentCircleRadius = circleRadius;
    currentStrafeAngle = strafeAngle;

    //when robot spins (pure spinning)
    if(Math.abs(circleRadius)<Constants.spin_threshold){
      
        leftFrontDirection.setSelectedSensorPosition(Constants.spin_angle*Constants.pos_units_per_degree);
        leftBackDirection.setSelectedSensorPosition(-Constants.spin_angle*Constants.pos_units_per_degree);
        rightFrontDirection.setSelectedSensorPosition(-Constants.spin_angle*Constants.pos_units_per_degree);
        rightBackDirection.setSelectedSensorPosition(Constants.spin_angle*Constants.pos_units_per_degree);

        leftFrontThrust.set(ControlMode.PercentOutput, speed);
        leftBackThrust.set(ControlMode.PercentOutput, speed);
        rightFrontThrust.set(ControlMode.PercentOutput, speed);
        rightBackThrust.set(ControlMode.PercentOutput, speed);
    
    }
    // strafing + turning
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
    //only strafing
    else{
        leftFrontThrust.set(ControlMode.PercentOutput, speed);
        leftBackThrust.set(ControlMode.PercentOutput, speed);
        rightFrontThrust.set(ControlMode.PercentOutput, speed);
        rightBackThrust.set(ControlMode.PercentOutput, speed);
    }
   odometry.updatePosition(); 
  }

  public double[] calcAngles(double circleRadius){

    double[] result = new double[8];

    //line going through robot center in the direction of the strafe angle
    Line robotCenterLinePerpendicular = new Line(currentStrafeAngle+90,Constants.center);
    
    //center of turning circle
    double[] circleCenter = {Math.cos(Math.toRadians(currentStrafeAngle+90))*circleRadius,  Math.sin(Math.toRadians(currentStrafeAngle+90))*circleRadius};
    currentCircleCenter = circleCenter;


    Line leftFront = new Line(currentStrafeAngle,Constants.leftFrontCenter);
    Line leftBack = new Line(currentStrafeAngle, Constants.leftBackCenter);
    Line rightFront = new Line(currentStrafeAngle,Constants.rightFrontCenter);
    Line rightBack = new Line(currentStrafeAngle, Constants.rightBackCenter);

    //poitns of intersection between each line above and the robot center line.
    double[] leftFrontP = robotCenterLinePerpendicular.getIntersection(leftFront);
    double[] leftBackP = robotCenterLinePerpendicular.getIntersection(leftBack);
    double[] rightFrontP = robotCenterLinePerpendicular.getIntersection(rightFront);
    double[] rightBackP = robotCenterLinePerpendicular.getIntersection(rightBack);

    //distance between circle center and wheel center
    double leftFrontRadius = RobotContainer.distance(leftFrontP, circleCenter);
    double leftBackRadius = RobotContainer.distance(leftBackP, circleCenter);
    double rightFrontRadius = RobotContainer.distance(rightFrontP, circleCenter);
    double rightBackRadius = RobotContainer.distance(rightBackP, circleCenter);


    currentWheelDistanceFromCircleCenter[0] = leftFrontRadius;
    currentWheelDistanceFromCircleCenter[1] = leftBackRadius;
    currentWheelDistanceFromCircleCenter[2] = rightFrontRadius;
    currentWheelDistanceFromCircleCenter[3] = rightBackRadius;

    double maxRadius  = Math.max(Math.max(Math.max(leftFrontRadius, leftBackRadius),rightFrontRadius),rightBackRadius);

  // result's order is leftF angle (0), leftB angle (1), rightF angle (2), rightB angle(3), 
  //leftF thrust (4), leftB thrust (5), rightF thrust (6), rightB thrust (7)
    result[0] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.leftFrontCenter, leftFrontP), leftFrontRadius))) * (circleRadius>0 ? -1:1);
    result[1] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.leftBackCenter, leftBackP), leftBackRadius))) * (circleRadius>0 ? 1:-1);
    result[2] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.rightFrontCenter, rightFrontP), rightFrontRadius))) * (circleRadius>0 ? -1:1);
    result[3] = currentStrafeAngle + RobotContainer.to360(Math.toDegrees(Math.atan2(RobotContainer.distance(Constants.rightBackCenter, rightBackP),rightBackRadius))) * (circleRadius>0 ? 1:-1);
    
    //thrust = radius/maxradius * speed
    result[4] = leftFrontRadius/maxRadius;
    result[5] = leftBackRadius/maxRadius;
    result[6] = rightFrontRadius/maxRadius;
    result[7] = rightBackRadius/maxRadius;
    return result; 
  }

  

  //setters
  public void setThrustSpeeds(double[] speeds){
    leftFrontThrust.set(ControlMode.PercentOutput, speeds[0]);
    leftBackThrust.set(ControlMode.PercentOutput, speeds[1]);
    rightFrontThrust.set(ControlMode.PercentOutput, speeds[2]);
    rightBackThrust.set(ControlMode.PercentOutput, speeds[3]);
  }
  public void setDirectionalAngles(double[] angles){
    double[] currentAngles = getAngles();
    double[] currentPositions = getDirectionalPositions();
    
    leftFrontDirection.set(TalonSRXControlMode.Position, 
    (currentPositions[0] + 
    RobotContainer.angleDistance2(angles[0], currentAngles[0])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[0], angles[0]) ? 1:-1)));
    
    leftBackDirection.set(TalonSRXControlMode.Position, 
    (currentPositions[1] + 
    RobotContainer.angleDistance2(angles[1], currentAngles[1])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[1], angles[1]) ? 1:-1)));
   
    rightFrontDirection.set(TalonSRXControlMode.Position, 
    (currentPositions[2] + 
    RobotContainer.angleDistance2(angles[2], currentAngles[2])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[2], angles[2]) ? 1:-1)));
   
    rightBackDirection.set(TalonSRXControlMode.Position, 
    (currentPositions[3] + 
    RobotContainer.angleDistance2(angles[3], currentAngles[3])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[3], angles[3]) ? 1:-1)));
  }
  //getters
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
  public double[] getAngles(){
    double[] result = getDirectionalPositions();
    for(int i = 0; i<result.length; i++){
      result[i] = RobotContainer.floorMod(result[i]/Constants.pos_units_per_degree, 360);
    }
    return result;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //corrects angles of directional motor encoders
    
    if(leftFrontDirection.getSelectedSensorPosition()/Constants.pos_units_per_degree>360)
     leftFrontDirection.setSelectedSensorPosition(leftFrontDirection.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(leftFrontDirection.getSelectedSensorPosition()<0)
     leftFrontDirection.setSelectedSensorPosition((360*Constants.pos_units_per_degree+leftFrontDirection.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

    if(leftBackDirection.getSelectedSensorPosition()/Constants.pos_units_per_degree>360)
     leftBackDirection.setSelectedSensorPosition(leftBackDirection.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(leftBackDirection.getSelectedSensorPosition()<0)
     leftBackDirection.setSelectedSensorPosition((360*Constants.pos_units_per_degree+leftBackDirection.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

    if(rightFrontDirection.getSelectedSensorPosition()/Constants.pos_units_per_degree>360)
     rightFrontDirection.setSelectedSensorPosition(rightFrontDirection.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(rightFrontDirection.getSelectedSensorPosition()<0)
      rightFrontDirection.setSelectedSensorPosition((360*Constants.pos_units_per_degree+rightFrontDirection.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

    if(rightBackDirection.getSelectedSensorPosition()/Constants.pos_units_per_degree>360) 
      rightBackDirection.setSelectedSensorPosition(rightBackDirection.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(rightBackDirection.getSelectedSensorPosition()<0) 
     rightBackDirection.setSelectedSensorPosition((360*Constants.pos_units_per_degree+rightBackDirection.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

  }
}
