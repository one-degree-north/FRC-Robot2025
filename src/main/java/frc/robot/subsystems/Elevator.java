// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//two options: make it so envoder resets when magnetic switch is active (make sure it only happens once), or use switch same as coral mech

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.CoralConstants;
import frc.robot.constants.RegularConstants.ElevatorConstants;
import frc.robot.subsystems.CoralMech.CoralStates;

public class Elevator extends SubsystemBase {
  private TalonFX m_leftElevatorMotor;
  private TalonFX m_rightElevatorMotor;

  private VoltageOut voltageOut = new VoltageOut(0);
  private DutyCycleEncoder m_elevatorEncoder;
  private ElevatorStates currentState;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private DigitalInput bottomLimitSwitch; // Magnetic Limit switch
  private boolean isElevatorEncoderReset;



  /** Creates a new Elevator. */
  public Elevator() {
    setName("Elevator");
    m_leftElevatorMotor = new TalonFX(ElevatorConstants.leftElevatorID, "rio");
    m_rightElevatorMotor = new TalonFX(ElevatorConstants.rightElevatorID, "rio");
    m_elevatorEncoder = new DutyCycleEncoder(ElevatorConstants.elevatorEncoderID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.magneticLimitSwitchID);
    motorConfigurations();
  }

  private void motorConfigurations(){
    
    TalonFXConfiguration leftElevatorConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("KrakenX60"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive))
    .withSlot0(MotorConfigs.getSlot0Configs(
      ElevatorConstants.leftElevatorkP, ElevatorConstants.leftElevatorkI, ElevatorConstants.leftElevatorkD, ElevatorConstants.leftElevatorkS, ElevatorConstants.leftElevatorkV, ElevatorConstants.leftElevatorkA, ElevatorConstants.leftElevatorkG))
    .withFeedback(MotorConfigs.getFeedbackConfigs(ElevatorConstants.leftElevatorMechanismRatio))
    .withMotionMagic(MotorConfigs.geMotionMagicConfigs(ElevatorConstants.leftElevatorMMAcceleration,
     ElevatorConstants.leftElevatorMMCruiseVelocity, ElevatorConstants.leftElevatorMMJerk));

     TalonFXConfiguration rightElevatorConfigs = new TalonFXConfiguration()
     .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("KrakenX60"))
     .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
       NeutralModeValue.Brake, InvertedValue.Clockwise_Positive))
     .withSlot0(MotorConfigs.getSlot0Configs(
       ElevatorConstants.rightElevatorkP, ElevatorConstants.rightElevatorkI, ElevatorConstants.rightElevatorkD, ElevatorConstants.rightElevatorkS, ElevatorConstants.rightElevatorkV, ElevatorConstants.rightElevatorkA, ElevatorConstants.rightElevatorkG))
     .withFeedback(MotorConfigs.getFeedbackConfigs(ElevatorConstants.rightElevatorMechanismRatio))
     .withMotionMagic(MotorConfigs.geMotionMagicConfigs(ElevatorConstants.rightElevatorMMAcceleration,
      ElevatorConstants.rightElevatorMMCruiseVelocity, ElevatorConstants.rightElevatorMMJerk));

      m_leftElevatorMotor.getConfigurator().apply(leftElevatorConfigs);
      m_rightElevatorMotor.getConfigurator().apply(rightElevatorConfigs);

  }
 

  public boolean isElevatorDown(){
    return bottomLimitSwitch.get();
  }


  private void setControl(TalonFX motor, ControlRequest req) {
  if (motor.isAlive() && isElevatorEncoderReset) {
    motor.setControl(req);
  }
  }

  public void setElevatorMotorsVoltage(double voltage){
    setControl(m_leftElevatorMotor, voltageOut.withOutput(voltage));
    setControl(m_rightElevatorMotor, voltageOut.withOutput(voltage));
  }

  private void zeroElevator(){
    m_leftElevatorMotor.setPosition(0).isOK();
    currentState = ElevatorStates.ELEVATOR_DOCKED;
  }

  public Boolean isElevatorAtSetpoint() { // FIX
    return Math.abs(m_elevatorEncoder.get() -
      currentState.getSetpointValue()) < CoralConstants.wristAllowedError;
  }
  

  private void stopMotor(TalonFX motor) {
    motor.stopMotor();
  }


  public void coralTransitionHandler(ElevatorStates wantedState) {
    switch (wantedState) {
        case ELEVATOR_UP, ELEVATOR_DOWN -> 
          setElevatorMotorsVoltage(wantedState.getSetpointValue()); //Does this need to change positive/negative for going up and down
        case ELEVATOR_DOCKED, ELEVATOR_L1, ELEVATOR_L2, ELEVATOR_L3, ELEVATOR_L4 -> 
          setControl(m_leftElevatorMotor, motionMagicVoltage.withPosition(wantedState.getSetpointValue()));
    }
    currentState = wantedState;
}

public enum ElevatorStates {
  ELEVATOR_UP(ElevatorConstants.elevatorUpVoltage),
  ELEVATOR_DOWN(ElevatorConstants.elevatorDownVoltage),
  ELEVATOR_DOCKED(ElevatorConstants.elevatorDockedPos),
  ELEVATOR_L1(ElevatorConstants.elevatorL1Pos),
  ELEVATOR_L2(ElevatorConstants.elevatorL2Pos),
  ELEVATOR_L3(ElevatorConstants.elevatorL3Pos),
  ELEVATOR_L4(ElevatorConstants.elevatorL4Pos);

  private final double setpointValue;

  ElevatorStates(double setpointValue) {
    this.setpointValue = setpointValue;
  }

  public double getSetpointValue() {
    return setpointValue;
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled() && bottomLimitSwitch.get()){
        zeroElevator();
    }
    SmartDashboard.putString("ElevatorState", currentState.name());
  }
}
