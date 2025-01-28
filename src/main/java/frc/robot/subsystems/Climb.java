// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.ClimbConstants;

public class Climb extends SubsystemBase {
  private TalonFX climbMotor;
  private VoltageOut voltageOut = new VoltageOut(0);
  
  public Climb() {
    climbMotor = new TalonFX(0, "rio");
    configureClimb();
  }

  private void configureClimb(){
     TalonFXConfiguration climbConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Coast, InvertedValue.Clockwise_Positive))
    .withFeedback(MotorConfigs.getFeedbackConfigs(ClimbConstants.climbMechanismRatio));

    climbMotor.getConfigurator().apply(climbConfigs);
  }

  private void setControl(TalonFX motor, ControlRequest req) {
    if (motor.isAlive()) {
      motor.setControl(req);
    }
 }

  public void climbUp(){
    setControl(climbMotor, voltageOut.withOutput(ClimbConstants.climbVoltageOut));
  }

  public void climbDown(){
    setControl(climbMotor, voltageOut.withOutput(-ClimbConstants.climbVoltageOut));
  }

  public void stopClimb(){
    setControl(climbMotor, voltageOut.withOutput(0));
  }

  public double getVelocity(){
    return climbMotor.getVelocity().getValueAsDouble();
  }

  public void climbTransitionHandler(ClimbStates state){
    switch(state){
      case CLIMBUP:
        climbUp();
        break;
      case CLIMBDOWN:
        climbDown();
        break;
    }
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Velocity", getVelocity());
  }

  public enum ClimbStates{
    CLIMBUP,CLIMBDOWN
  }
}
