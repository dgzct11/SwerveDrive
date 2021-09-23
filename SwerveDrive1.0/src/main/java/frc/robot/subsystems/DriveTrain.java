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
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

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

  public TalonSRX lfd = new TalonSRX(Constants.left_front_direction_port);
  public TalonSRX lft = new TalonSRX(Constants.left_front_thrust_port);

  public TalonSRX rfd = new TalonSRX(Constants.right_front_direction_port);
  public TalonSRX rft = new TalonSRX(Constants.right_front_thrust_port);

  public TalonSRX lbd = new TalonSRX(Constants.left_back_direction_port);
  public TalonSRX lbt = new TalonSRX(Constants.left_back_thrust_port);

  public TalonSRX rbd = new TalonSRX(Constants.right_back_direction_port);
  public TalonSRX rbt = new TalonSRX(Constants.right_back_thrust_port);

  public TalonSRX[] directionals = {lfd, lbd, rfd, rbd};
  public TalonSRX[] thrusts = {lft, lbt, rft, rbt};

  public double kpDir = 0.01;
  public double kiDir = 0;
  public double kdDir = 0.01;
  public double kfDir = 0.15;
  public int slotIdx = 0;
  int timeout = 0;
  double errorDeg = 1;
  double motionVelociy = 500;
  double motionAcceleration = 1000;

  PIDControl directionalPIDLF = new PIDControl(kpDir, kiDir, kdDir);
  PIDControl directionalPIDLB = new PIDControl(kpDir, kiDir, kdDir);
  PIDControl directionalPIDRF = new PIDControl(kpDir, kiDir, kdDir);
  PIDControl directionalPIDRB = new PIDControl(kpDir, kiDir, kdDir);

  public DriveTrain() {
    for(int i = 0; i<4; i++){
      TalonSRX motor = directionals[i];
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setInverted(true);
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
      TalonSRX motor = thrusts[i];
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
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
    setDirectionalAngles(angles);
    //setThrustSpeeds(speeds);
    SmartDashboard.putNumber("LF turnto", angles[0]);
    SmartDashboard.putNumber("LB turnto", angles[1]);
    SmartDashboard.putNumber("RF turnto", angles[2]);
    SmartDashboard.putNumber("RB turnto", angles[3]);
  }
  public void setThrustSpeeds(double[] speeds){
    lft.set(ControlMode.PercentOutput, speeds[0]);
    lbt.set(ControlMode.PercentOutput, speeds[1]);
    rft.set(ControlMode.PercentOutput, speeds[2]);
    rbt.set(ControlMode.PercentOutput, speeds[3]);
  }
  public void setDirectionalAnglesPID(double[] angles){
    double[] currentAngles = getAngles();
    directionalPIDLF.setSetpoint(angles[0], currentAngles[0]);
    directionalPIDLB.setSetpoint(angles[1], currentAngles[1]);
    directionalPIDRF.setSetpoint(angles[2], currentAngles[2]);
    directionalPIDRB.setSetpoint(angles[3], currentAngles[3]);

    if(RobotContainer.angleDistance2(currentAngles[0], angles[0]) >= errorDeg)
     lfd.set(TalonSRXControlMode.PercentOutput, (RobotContainer.shouldTurnLeft(angles[0], currentAngles[0])?1:-1)*directionalPIDLF.getOutput(currentAngles[0]));
    else lfd.set(TalonSRXControlMode.PercentOutput, 0);
    
    if(RobotContainer.angleDistance2(currentAngles[1], angles[1]) >= errorDeg)
      lbd.set(TalonSRXControlMode.PercentOutput, (RobotContainer.shouldTurnLeft(angles[1], currentAngles[1])?1:-1)*directionalPIDLB.getOutput(currentAngles[1]));
    else lbd.set(TalonSRXControlMode.PercentOutput, 0);
   
    if(RobotContainer.angleDistance2(currentAngles[2], angles[2]) >= errorDeg)
      rfd.set(TalonSRXControlMode.PercentOutput, (RobotContainer.shouldTurnLeft(angles[2], currentAngles[2])?1:-1)*directionalPIDRF.getOutput(currentAngles[2]));
    else rfd.set(TalonSRXControlMode.PercentOutput, 0);
   
    if(RobotContainer.angleDistance2(currentAngles[3], angles[3]) >= errorDeg)
      rbd.set(TalonSRXControlMode.PercentOutput, (RobotContainer.shouldTurnLeft(angles[3], currentAngles[3])?1:-1)*directionalPIDRB.getOutput(currentAngles[3]));
    else rbd.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void setDirectionalAngles(double[] angles){
    double[] currentAngles = getAngles();
    SmartDashboard.putNumber("LF Diff", RobotContainer.angleDistance2(currentAngles[0], angles[0]));
    SmartDashboard.putNumber("LB Diff", RobotContainer.angleDistance2(currentAngles[1], angles[1]));
    SmartDashboard.putNumber("RF Diff", RobotContainer.angleDistance2(currentAngles[2], angles[2]));
    SmartDashboard.putNumber("RB Diff", RobotContainer.angleDistance2(currentAngles[3], angles[3]));
    for(int i = 0; i<4; i++){
      TalonSRX motor = directionals[i];
      //if(RobotContainer.angleDistance2(currentAngles[3], angles[3]) >= errorDeg)
        motor.set(TalonSRXControlMode.Position, 
              ((motor.getSelectedSensorPosition() + 
              RobotContainer.angleDistance2(angles[i], currentAngles[i])*Constants.pos_units_per_degree * 
              (RobotContainer.shouldTurnLeft(currentAngles[i], angles[i]) ? -1:1))));
      //else rbd.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  public double[] getDirectionalPositions(){
    double[] result = new double[4];
    for(int i = 0; i<4; i++)
      result[i] = directionals[i].getSelectedSensorPosition();
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
    /*
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
    */
  }
}
