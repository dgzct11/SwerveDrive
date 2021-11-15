// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * jadf;laskdjf
 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;


public class DriveTrain extends SubsystemBase {
  private ShuffleboardTab tab = Shuffleboard.getTab("PID DriveTrain Constants");
  
  private NetworkTableEntry kpDirEntry = tab.add("Directional KP", 0).getEntry();
  private NetworkTableEntry kiDirEntry = tab.add("Directional KI", 0).getEntry();
  private NetworkTableEntry kdDirEntry = tab.add("Directional KD", 0).getEntry();
 
  private NetworkTableEntry kpThPosEntry = tab.add("Thrust KP Pos", 0).getEntry();
  private NetworkTableEntry kiThPosEntry = tab.add("Thrust KI Pos", 0).getEntry();
  private NetworkTableEntry kdThPosEntry = tab.add("Thrust KD Pos", 0).getEntry();


  private NetworkTableEntry kpThEntry = tab.add("Thrust KP", 0).getEntry();
  private NetworkTableEntry kiThEntry = tab.add("Thrust KI", 0).getEntry();
  private NetworkTableEntry kdThEntry = tab.add("Thrust KD", 0).getEntry();
  private NetworkTableEntry kfThEntry = tab.add("Thrust KF", 0).getEntry();


  public TalonFX lfd = new TalonFX(Constants.left_front_direction_port);
  public TalonFX lft = new TalonFX(Constants.left_front_thrust_port);

  public TalonFX rfd = new TalonFX(Constants.right_front_direction_port);
  public TalonFX rft = new TalonFX(Constants.right_front_thrust_port);

  public TalonFX lbd = new TalonFX(Constants.left_back_direction_port);
  public TalonFX lbt = new TalonFX(Constants.left_back_thrust_port);

  public TalonFX rbd = new TalonFX(Constants.right_back_direction_port);
  public TalonFX rbt = new TalonFX(Constants.right_back_thrust_port);

  public TalonFX[] directionals = {lfd, lbd, rfd, rbd};
  public TalonFX[] thrusts = {lft, lbt, rft, rbt};

  //dir index 0
  public double kpDir = 0.2;
  public double kiDir = 0.002;
  public double kdDir = 0;
 
  double errorDeg = 0.01;

  //thrust index 1 position
  public double kpThPos = 0.2;
  public double kiThPos = 0;
  public double kdThPos = 0;
  double errorPos = 10;
//thrust index 0 velocity
  public double kpTh = 0.01;
  public double kiTh = 0;
  public double kdTh = 0;
  public double kfTh = 0.045;

  //algining with angle constant
  public double alignKP = 0.03;
  public double alignAngleSpeed = 0.3;
  public int[] thrustCoefficients = {1,1,1,1};

  int timeout = 0;
 
  double motionVelociy = 500;
  double motionAcceleration = 1000;



  public DriveTrain() {
    NavXGyro.ahrs.reset();

    //dir motors
    for(int i = 0; i<4; i++){
      TalonFX dirMotor = directionals[i];
      dirMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      
      dirMotor.configAllowableClosedloopError(0, errorDeg*Constants.pos_units_per_degree);
    
      dirMotor.setNeutralMode(NeutralMode.Brake);
     

      dirMotor.config_kP(0, kpDir);
      dirMotor.config_kI(0, kiDir);
      dirMotor.config_kD(0, kdDir);

    }

    //thrust motors
    for(int i = 0; i<4; i++){
      TalonFX thMotor = thrusts[i];
      thMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      thMotor.config_kP(1, kpThPos);
      thMotor.config_kI(1, kiThPos);
      thMotor.config_kD(1, kdThPos);
      
      thMotor.configAllowableClosedloopError(0, errorPos, 0);

      thMotor.config_kP(0, kpTh);
      thMotor.config_kI(0, kiTh);
      thMotor.config_kD(0, kdTh);
      thMotor.config_kF(0, kfTh);
      thMotor.setNeutralMode(NeutralMode.Brake);
    }

    rft.setInverted(true);
    rbt.setInverted(true);
     
    lfd.setSelectedSensorPosition(0);
    lft.setSelectedSensorPosition(0);
    lbd.setSelectedSensorPosition(0);
    lbt.setSelectedSensorPosition(0);
    rfd.setSelectedSensorPosition(0);
    rft.setSelectedSensorPosition(0);
    rbd.setSelectedSensorPosition(0);
    rbt.setSelectedSensorPosition(0);
    
  }

  
  public void alignDrive(double strafeAngle, double speed, double angle){
    double error = Math.min( alignKP*RobotContainer.angleDistance2(angle, NavXGyro.getAngle()), alignAngleSpeed) *(RobotContainer.shouldTurnLeft( NavXGyro.getAngle(), angle) ? 1:-1) ;
    SmartDashboard.putNumber("AD Error", error);
    
    fieldOrientedDrive(strafeAngle, speed, error );
  }
  public void driveDistance(double angle, double distance){
    distance *= Constants.pos_units_per_meter;
    double[] angles = {angle, angle, angle, angle};
    setDirectionalAngles(angles);
    
    for(short i = 0; i<4; i++){
      TalonFX motor = thrusts[i];
      motor.selectProfileSlot(1, 0);
      double newPos = motor.getSelectedSensorPosition() + distance*thrustCoefficients[i];
      motor.set(TalonFXControlMode.Position, newPos);
      
    }
  }
  public void fieldOrientedDrive(double strafeAngle, double speed, double rotateSpeed){
    strafeAngle = (RobotContainer.angleDistance2(NavXGyro.getAngle(), strafeAngle) * (RobotContainer.shouldTurnLeft(NavXGyro.getAngle(), strafeAngle)?1:-1) + 360 ) %360;
    rotateDriveVelocity(strafeAngle, speed, rotateSpeed);
  }
  public void rotateDriveVelocity(double strafeAngle, double speed, double rotateSpeed){
    //positive rotate speed is left turn, negative rotate speed is right turn
    double strafeXComponent = -Math.sin(Math.toRadians(strafeAngle))*speed;
    double strafeYComponent = Math.cos(Math.toRadians(strafeAngle))*speed;
    double rotationComponent = Constants.rotate_dampaner*rotateSpeed/Math.sqrt(2);;

    double[] leftFrontVector = {strafeXComponent-rotationComponent, strafeYComponent-rotationComponent};
    double[] leftBackVector = {strafeXComponent + rotationComponent, strafeYComponent-rotationComponent};
    double[] rightFrontVector = {strafeXComponent-rotationComponent, strafeYComponent + rotationComponent};
    double[] rightBackVector = {strafeXComponent + rotationComponent, strafeYComponent + rotationComponent};
     double[] angles = {RobotContainer.stickTo360(leftFrontVector[0], leftFrontVector[1]),
                       RobotContainer.stickTo360(leftBackVector[0], leftBackVector[1]),
                       RobotContainer.stickTo360(rightFrontVector[0], rightFrontVector[1]),
                       RobotContainer.stickTo360(rightBackVector[0], rightBackVector[1])};
    double[] speeds = {RobotContainer.magnitutde(leftFrontVector),
                       RobotContainer.magnitutde(leftBackVector),
                       RobotContainer.magnitutde(rightFrontVector),
                       RobotContainer.magnitutde(rightBackVector)};
    setDirectionalAnglesEff(angles); //TODO add Eff
   
    setVelocities(speeds);
 
  }

  public void setThrustSpeeds(double[] speeds){
    lft.set(ControlMode.PercentOutput, thrustCoefficients[0] * Constants.max_motor_percent*speeds[0]);
    lbt.set(ControlMode.PercentOutput,  thrustCoefficients[1] * Constants.max_motor_percent*speeds[1]);
    rft.set(ControlMode.PercentOutput,  thrustCoefficients[2] * Constants.max_motor_percent*speeds[2]);
    rbt.set(ControlMode.PercentOutput, thrustCoefficients[3] * Constants.max_motor_percent*speeds[3]);
  }
  public void setVelocities(double[] speeds){
    for(short i = 0; i<4; i++){
      TalonFX motor = thrusts[i];
      motor.selectProfileSlot(0, 0);
      motor.set(ControlMode.Velocity,  Constants.velocityMax * thrustCoefficients[i] * Constants.talon_velocity_per_ms*speeds[i]);
    }
    /*lft.set(ControlMode.Velocity,  Constants.velocityMax * thrustCoefficients[0] * Constants.talon_velocity_per_ms*speeds[0]);
    lbt.set(ControlMode.Velocity,  Constants.velocityMax * thrustCoefficients[1] * Constants.talon_velocity_per_ms*speeds[1]);
    rft.set(ControlMode.Velocity,  Constants.velocityMax * thrustCoefficients[2] * Constants.talon_velocity_per_ms*speeds[2]);
    rbt.set(ControlMode.Velocity,  Constants.velocityMax * thrustCoefficients[3] * Constants.talon_velocity_per_ms*speeds[3]);*/
    
    double[] currentSpeeds = getVelocities();
    SmartDashboard.putNumber("LF V Diff", Math.abs(currentSpeeds[0] - speeds[0]));
    SmartDashboard.putNumber("LB V Diff", Math.abs(currentSpeeds[1] - speeds[1]));
    SmartDashboard.putNumber("RF V Diff", Math.abs(currentSpeeds[2] - speeds[2]));
    SmartDashboard.putNumber("RB V Diff", Math.abs(currentSpeeds[3] - speeds[3]));
  }
  public int[] getThrustCoefficients(){
    return thrustCoefficients;
  }
  public void setDirectionalAngles(double[] angles){
    double[] currentAngles = getAngles();
    SmartDashboard.putNumber("LF A Diff", RobotContainer.angleDistance2(currentAngles[0], angles[0]));
    SmartDashboard.putNumber("LB A Diff", RobotContainer.angleDistance2(currentAngles[1], angles[1]));
    SmartDashboard.putNumber("RF A Diff", RobotContainer.angleDistance2(currentAngles[2], angles[2]));
    SmartDashboard.putNumber("RB A Diff", RobotContainer.angleDistance2(currentAngles[3], angles[3]));
    for(int i = 0; i<4; i++){
      TalonFX motor = directionals[i];
        motor.set(TalonFXControlMode.Position, 
              ((motor.getSelectedSensorPosition() + 
              RobotContainer.angleDistance2(angles[i], currentAngles[i])*Constants.pos_units_per_degree * 
              (RobotContainer.shouldTurnLeft(currentAngles[i], angles[i]) ? 1:-1)))); 
    }
  }
  public void stop(){
    double[] speeds = {0,0,0,0};
    setThrustSpeeds(speeds);
  }
  public void setDirectionalAnglesEff(double[] angles){
    double[] currentAngles = getAngles();
    for(int i = 0; i<4; i++){
      TalonFX motor = directionals[i];
      if(RobotContainer.angleDistance2(angles[i], currentAngles[i]) > 90){
        thrustCoefficients[i] = -1;
        angles[i] = (angles[i]+180)%360;
      }
      else
        thrustCoefficients[i] = 1;
      motor.set(TalonFXControlMode.Position, 
              ((motor.getSelectedSensorPosition() + 
              RobotContainer.angleDistance2(angles[i], currentAngles[i])*Constants.pos_units_per_degree * 
              (RobotContainer.shouldTurnLeft(currentAngles[i], angles[i]) ? 1:-1))));
    }
    SmartDashboard.putNumber("LF A Diff", RobotContainer.angleDistance2(currentAngles[0], angles[0]));
    SmartDashboard.putNumber("LB A Diff", RobotContainer.angleDistance2(currentAngles[1], angles[1]));
    SmartDashboard.putNumber("RF A Diff", RobotContainer.angleDistance2(currentAngles[2], angles[2]));
    SmartDashboard.putNumber("RB A Diff", RobotContainer.angleDistance2(currentAngles[3], angles[3]));
  }

  public double[] getDirectionalPositions(){
    double[] result = new double[4];
    for(int i = 0; i<4; i++)
      result[i] = directionals[i].getSelectedSensorPosition();
    return result;
  }
 
  public double[] getThrustPositions(){
    double[] result = new double[4];
    for(int i = 0; i<4; i++)
      result[i] = thrusts[i].getSelectedSensorPosition();
    return result;
  }
  public double[] getAngles(){
    double[] result = getDirectionalPositions();
    for(int i = 0; i<result.length; i++){
      result[i] = RobotContainer.floorMod(result[i]/Constants.pos_units_per_degree, 360);
    }
    return result;
  }
  public double[] getVelocities(){
    double[] result = new double[4];
    for(int i = 0; i<result.length; i++){
      result[i] = thrusts[i].getSelectedSensorVelocity() / Constants.talon_velocity_per_ms;
    }
    return result;
  }
  public void setDirectionalConstants(){
    kpDir = kpDirEntry.getDouble(kpDir);
    kiDir = kiDirEntry.getDouble(kiDir);
    kdDir = kdDirEntry.getDouble(kdDir);
    for(int i = 0; i<4; i++){
      TalonFX dirMotor = directionals[i];
      dirMotor.config_kI(0, kiDir);
      dirMotor.config_kP(0, kpDir);
      dirMotor.config_kD(0, kdDir);
    }
  }
  public void setThrustConstants(){
    kpThPos = kpThPosEntry.getDouble(kpThPos);
    kiThPos = kiThPosEntry.getDouble(kiThPos);
    kdThPos = kdThPosEntry.getDouble(kdThPos);

    kpTh = kpThEntry.getDouble(kpTh);
    kiTh = kiThEntry.getDouble(kiTh);
    kdTh = kdThEntry.getDouble(kdTh);
    kfTh = kfThEntry.getDouble(kfTh);

    for(short i = 0; i<4; i ++){
      TalonFX thMotor = thrusts[i];
      thMotor.config_kP(1, kpThPos);
      thMotor.config_kI(1, kiThPos);
      thMotor.config_kD(1, kdThPos);

      thMotor.config_kP(0, kpTh);
      thMotor.config_kI(0, kiTh);
      thMotor.config_kD(0, kdTh);
      thMotor.config_kF(0, kfTh);
    }
   
  }
  public void displayValues(){
    SmartDashboard.putNumber("Drive Mode", Constants.drive_mode);
    SmartDashboard.putNumber("Max V", Constants.velocityMax);

    //Angle difference in set Directional angles
    //Velocity difference in set Veloctiy

    double[] velocities = getVelocities();
    SmartDashboard.putNumber("LF V", velocities[0]);
    SmartDashboard.putNumber("LB V", velocities[1]);
    SmartDashboard.putNumber("RF V", velocities[2]);
    SmartDashboard.putNumber("RB V", velocities[3]);

    double[] angles = getAngles();
    SmartDashboard.putNumber("LF A", angles[0]);
    SmartDashboard.putNumber("LB A", angles[1]);
    SmartDashboard.putNumber("RF A", angles[2]);
    SmartDashboard.putNumber("RB A", angles[3]);
  }
  @Override
  public void periodic() {
    
    
    displayValues();
    //setConstants();
    //setDirectionalConstants();
    
    
  }
}
