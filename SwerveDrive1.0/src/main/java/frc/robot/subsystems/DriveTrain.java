// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * jadf;laskdjf
 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import frc.robot.functional.PIDControl;

public class DriveTrain extends SubsystemBase {
  private ShuffleboardTab tab = Shuffleboard.getTab("PID DriveTrain Constants");
  
  private NetworkTableEntry kpDirEntry = tab.add("Directional KP", 0).getEntry();
  private NetworkTableEntry kiDirEntry = tab.add("Directional KI", 0).getEntry();
  private NetworkTableEntry kdDirEntry = tab.add("Directional KD", 0).getEntry();
  private NetworkTableEntry kfDirEntry = tab.add("Directional KF", 0).getEntry();


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

  public double kpDir = 0.2;
  public double kiDir = 0.002;
  public double kdDir = 0;
  public double kfDir = 0;
  public int slotIdx = 0;

  public int[] thrustCoefficients = {1,1,1,1};

  int timeout = 0;
  double errorDeg = 0.01;
  double motionVelociy = 500;
  double motionAcceleration = 1000;

  PIDControl directionalPIDLF = new PIDControl(kpDir, kiDir, kdDir);
  PIDControl directionalPIDLB = new PIDControl(kpDir, kiDir, kdDir);
  PIDControl directionalPIDRF = new PIDControl(kpDir, kiDir, kdDir);
  PIDControl directionalPIDRB = new PIDControl(kpDir, kiDir, kdDir);

  public DriveTrain() {
    for(int i = 0; i<4; i++){
      TalonFX motor = directionals[i];
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      
      motor.configAllowableClosedloopError(slotIdx, errorDeg*Constants.pos_units_per_degree);
    
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configMotionCruiseVelocity(motionVelociy);
      motor.configMotionAcceleration(motionAcceleration);

      motor.config_kP(slotIdx, kpDir);
      motor.config_kI(slotIdx, kiDir);
      motor.config_kD(slotIdx, kdDir);
      motor.config_kF(slotIdx, kfDir);

      
    }
    
    for(int i = 0; i<4; i++){
      TalonFX motor = thrusts[i];
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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

  public void rotateDrive(double strafeAngle, double speed, double rotateSpeed){
    //positive rotate speed is left turn, negative rotate speed is right turn
    double strafeXComponent = -Math.sin(Math.toRadians(strafeAngle))*speed;
    double strafeYComponent = Math.cos(Math.toRadians(strafeAngle))*speed;
    double rotationComponent = -Constants.rotate_dampaner*rotateSpeed/Math.sqrt(2);;

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
    setDirectionalAngles(angles);
    setThrustSpeeds(speeds);
    SmartDashboard.putNumber("LF turnto", angles[0]);
    SmartDashboard.putNumber("LB turnto", angles[1]);
    SmartDashboard.putNumber("RF turnto", angles[2]);
    SmartDashboard.putNumber("RB turnto", angles[3]);
  }
  public void setThrustSpeeds(double[] speeds){
    lft.set(ControlMode.PercentOutput, thrustCoefficients[0] * Constants.max_motor_percent*speeds[0]);
    lbt.set(ControlMode.PercentOutput,  thrustCoefficients[1] * Constants.max_motor_percent*speeds[1]);
    rft.set(ControlMode.PercentOutput,  thrustCoefficients[2] * Constants.max_motor_percent*speeds[2]);
    rbt.set(ControlMode.PercentOutput, thrustCoefficients[3] * Constants.max_motor_percent*speeds[3]);
  }
 

  public void setDirectionalAngles(double[] angles){
    double[] currentAngles = getAngles();
    SmartDashboard.putNumber("LF Diff", RobotContainer.angleDistance2(currentAngles[0], angles[0]));
    SmartDashboard.putNumber("LB Diff", RobotContainer.angleDistance2(currentAngles[1], angles[1]));
    SmartDashboard.putNumber("RF Diff", RobotContainer.angleDistance2(currentAngles[2], angles[2]));
    SmartDashboard.putNumber("RB Diff", RobotContainer.angleDistance2(currentAngles[3], angles[3]));
    for(int i = 0; i<4; i++){
      TalonFX motor = directionals[i];
      
        motor.set(TalonFXControlMode.Position, 
              ((motor.getSelectedSensorPosition() + 
              RobotContainer.angleDistance2(angles[i], currentAngles[i])*Constants.pos_units_per_degree * 
              (RobotContainer.shouldTurnLeft(currentAngles[i], angles[i]) ? 1:-1))));
      
    }
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
    SmartDashboard.putNumber("LF Diff", RobotContainer.angleDistance2(currentAngles[0], angles[0]));
    SmartDashboard.putNumber("LB Diff", RobotContainer.angleDistance2(currentAngles[1], angles[1]));
    SmartDashboard.putNumber("RF Diff", RobotContainer.angleDistance2(currentAngles[2], angles[2]));
    SmartDashboard.putNumber("RB Diff", RobotContainer.angleDistance2(currentAngles[3], angles[3]));
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

  @Override
  public void periodic() {
    double[] angles = getAngles();
    SmartDashboard.putNumber("LF Angle", angles[0]);
    SmartDashboard.putNumber("LB Angle", angles[1]);
    SmartDashboard.putNumber("RF Angle", angles[2]);
    SmartDashboard.putNumber("RB Angle", angles[3]);
    
    kpDir = kpDirEntry.getDouble(kpDir);
    kiDir = kiDirEntry.getDouble(kiDir);
    kdDir = kdDirEntry.getDouble(kdDir);
  
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
    
  }
}
