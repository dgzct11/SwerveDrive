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

import frc.robot.RobotContainer;
import frc.robot.functional.Line;

public class DriveTrain extends SubsystemBase {



  /** Creates a new DriveTrain. */
  public TalonSRX lfd = new TalonSRX(Constants.left_front_direction_port);
  public TalonSRX lft = new TalonSRX(Constants.left_front_thrust_port);

  public TalonSRX rfd = new TalonSRX(Constants.right_front_direction_port);
  public TalonSRX rft = new TalonSRX(Constants.right_front_thrust_port);

  public TalonSRX lbd = new TalonSRX(Constants.left_back_direction_port);
  public TalonSRX lbt = new TalonSRX(Constants.left_back_thrust_port);

  public TalonSRX rbd = new TalonSRX(Constants.right_back_direction_port);
  public TalonSRX rbt = new TalonSRX(Constants.right_back_thrust_port);

  //utility variables
  public double kpDir = 0;
  public double kiDir = 0;
  public double kdDir = 0;
  public double kfDir = 0;

  public double kpTh = 0;
  public double kiTh = 0;
  public double kdTh = 0;
  public double kfTh = 0;
  public int slotIdx = 1;
  //circle refers to circular path of rotation when turning
  public double currentStrafeAngle = 0;
  public double currentCircleRadius = 0;
  public boolean spinning = false;
  public double[] currentCircleCenter = new double[2];
  public double[] currentWheelDistanceFromCircleCenter = new double[4];





  public Odometry odometry;

  public DriveTrain(Odometry od) {
    //configure sensors for each motor controller to sensor in Falcon500
    lfd.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    lft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    lbd.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    lbt.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rfd.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rbd.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rbt.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    /*
    lfd.config_kP(slotIdx, kpDir);
    lbd.config_kP(slotIdx, kpDir);
    rfd.config_kP(slotIdx, kpDir);
    rbd.config_kP(slotIdx, kpDir);

    lfd.config_kI(slotIdx, kiDir);
    lbd.config_kI(slotIdx, kiDir);
    rfd.config_kI(slotIdx, kiDir);
    rbd.config_kI(slotIdx, kiDir);

    lfd.config_kD(slotIdx, kdDir);
    lbd.config_kD(slotIdx, kdDir);
    rfd.config_kD(slotIdx, kdDir);
    rbd.config_kD(slotIdx, kdDir);

    lfd.config_kF(slotIdx, kfDir);
    lbd.config_kF(slotIdx, kfDir);
    rfd.config_kF(slotIdx, kfDir);
    rbd.config_kF(slotIdx, kfDir);

    lft.config_kP(slotIdx, kpTh);
    lbt.config_kP(slotIdx, kpTh);
    rft.config_kP(slotIdx, kpTh);
    rbt.config_kP(slotIdx, kpTh);

    lft.config_kI(slotIdx, kiTh);
    lbt.config_kI(slotIdx, kiTh);
    rft.config_kI(slotIdx, kiTh);
    rbt.config_kI(slotIdx, kiTh);

    lft.config_kD(slotIdx, kdTh);
    lbt.config_kD(slotIdx, kdTh);
    rft.config_kD(slotIdx, kdTh);
    rbt.config_kD(slotIdx, kdTh);

    lft.config_kF(slotIdx, kfTh);
    lbt.config_kF(slotIdx, kfTh);
    rft.config_kF(slotIdx, kfTh);
    rbt.config_kF(slotIdx, kfTh);

*/
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
      
        lfd.setSelectedSensorPosition(Constants.spin_angle*Constants.pos_units_per_degree);
        lbd.setSelectedSensorPosition(-Constants.spin_angle*Constants.pos_units_per_degree);
        rfd.setSelectedSensorPosition(-Constants.spin_angle*Constants.pos_units_per_degree);
        rbd.setSelectedSensorPosition(Constants.spin_angle*Constants.pos_units_per_degree);

        lft.set(ControlMode.PercentOutput, speed);
        lbt.set(ControlMode.PercentOutput, speed);
        rft.set(ControlMode.PercentOutput, speed);
        rbt.set(ControlMode.PercentOutput, speed);
    
    }
    // strafing + turning
    else if(! (Math.abs(circleRadius) == Double.POSITIVE_INFINITY)){
        double[] angles_speeds = calcAngles(circleRadius);

        lfd.setSelectedSensorPosition((angles_speeds[0])*Constants.pos_units_per_degree);
        lbd.setSelectedSensorPosition(angles_speeds[1]*Constants.pos_units_per_degree);
        rfd.setSelectedSensorPosition(angles_speeds[2]*Constants.pos_units_per_degree);
        rbd.setSelectedSensorPosition(angles_speeds[3]*Constants.pos_units_per_degree);
      
        lft.set(ControlMode.PercentOutput, speed*angles_speeds[4]);
        lbt.set(ControlMode.PercentOutput, speed*angles_speeds[5]);
        rft.set(ControlMode.PercentOutput, speed*angles_speeds[6]);
        rbt.set(ControlMode.PercentOutput, speed*angles_speeds[7]);
      
    }
    //only strafing
    else{
        lft.set(ControlMode.PercentOutput, speed);
        lbt.set(ControlMode.PercentOutput, speed);
        rft.set(ControlMode.PercentOutput, speed);
        rbt.set(ControlMode.PercentOutput, speed);
    }
   
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
    lft.set(ControlMode.PercentOutput, speeds[0]);
    lbt.set(ControlMode.PercentOutput, speeds[1]);
    rft.set(ControlMode.PercentOutput, speeds[2]);
    rbt.set(ControlMode.PercentOutput, speeds[3]);
  }
  public void setDirectionalAngles(double[] angles){
    double[] currentAngles = getAngles();
    double[] currentPositions = getDirectionalPositions();
    
    lfd.set(TalonSRXControlMode.Position, 
    (currentPositions[0] + 
    RobotContainer.angleDistance2(angles[0], currentAngles[0])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[0], angles[0]) ? 1:-1)));
    
    lbd.set(TalonSRXControlMode.Position, 
    (currentPositions[1] + 
    RobotContainer.angleDistance2(angles[1], currentAngles[1])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[1], angles[1]) ? 1:-1)));
   
    rfd.set(TalonSRXControlMode.Position, 
    (currentPositions[2] + 
    RobotContainer.angleDistance2(angles[2], currentAngles[2])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[2], angles[2]) ? 1:-1)));
   
    rbd.set(TalonSRXControlMode.Position, 
    (currentPositions[3] + 
    RobotContainer.angleDistance2(angles[3], currentAngles[3])*Constants.pos_units_per_degree * 
    (RobotContainer.shouldTurnLeft(currentAngles[3], angles[3]) ? 1:-1)));
  }
  //getters
  public double[] getThrustPositions(){
    double[] result = new double[4];
    result[0] = lft.getSelectedSensorPosition();
    result[1] = lbt.getSelectedSensorPosition();
    result[2] = rft.getSelectedSensorPosition();
    result[3] = rbt.getSelectedSensorPosition();
    return result;

  }
   
  public double[] getDirectionalPositions(){
    double[] result = new double[4];
    result[0] = lfd.getSelectedSensorPosition();

    result[1] = lbd.getSelectedSensorPosition();
    result[2] = rfd.getSelectedSensorPosition();
    result[3] = rbd.getSelectedSensorPosition();
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
    
    if(lfd.getSelectedSensorPosition()/Constants.pos_units_per_degree>360)
     lfd.setSelectedSensorPosition(lfd.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(lfd.getSelectedSensorPosition()<0)
     lfd.setSelectedSensorPosition((360*Constants.pos_units_per_degree+lfd.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

    if(lbd.getSelectedSensorPosition()/Constants.pos_units_per_degree>360)
     lbd.setSelectedSensorPosition(lbd.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(lbd.getSelectedSensorPosition()<0)
     lbd.setSelectedSensorPosition((360*Constants.pos_units_per_degree+lbd.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

    if(rfd.getSelectedSensorPosition()/Constants.pos_units_per_degree>360)
     rfd.setSelectedSensorPosition(rfd.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(rfd.getSelectedSensorPosition()<0)
      rfd.setSelectedSensorPosition((360*Constants.pos_units_per_degree+rfd.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

    if(rbd.getSelectedSensorPosition()/Constants.pos_units_per_degree>360) 
      rbd.setSelectedSensorPosition(rbd.getSelectedSensorPosition()%(360*Constants.pos_units_per_degree));
    else if(rbd.getSelectedSensorPosition()<0) 
     rbd.setSelectedSensorPosition((360*Constants.pos_units_per_degree+rbd.getSelectedSensorPosition())%(360*Constants.pos_units_per_degree));

  }
}
