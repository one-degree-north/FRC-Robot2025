// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.AlgaeConstants;

@Logged
public class AlgaeMech extends SubsystemBase {
  private TalonFX m_leftFlywheel;
  private TalonFX m_rightFlywheel;
  private TalonFX m_innerRollers;
  private TalonFX m_leftPivot;
  private TalonFX m_rightPivot;

  private MotionMagicVelocityVoltage motionMagicVelocityVoltage 
  = new MotionMagicVelocityVoltage(0).withSlot(0);
  private VoltageOut voltageOut = new VoltageOut(0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private DigitalInput m_zeroSwitch;
  private boolean isPivotEncoderReset;
  private static DigitalInput algaeSensor;
  
    private AlgaeStates currentState;
  
    /** Creates a new AlgaeMech. */
    public AlgaeMech(DigitalInput zeroSwitch) {
      setName("AlgaeMech");
      m_leftFlywheel = new TalonFX(AlgaeConstants.leftFlywheelID, "rio");
      m_rightFlywheel = new TalonFX(AlgaeConstants.rightFlywheelID, "rio");
      m_innerRollers = new TalonFX(AlgaeConstants.innerRollersID, "rio");
      m_leftPivot = new TalonFX(AlgaeConstants.leftPivotID, "rio");
      m_rightPivot = new TalonFX(AlgaeConstants.rightPivotID, "rio");
      algaeSensor = new DigitalInput(AlgaeConstants.beamBreakID);
      m_zeroSwitch = zeroSwitch;
      configMotors();
    }
  
  private void configMotors(){
    TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive))
    .withFeedback(MotorConfigs.getFeedbackConfigs(AlgaeConstants.flywheelMechanismRatio))
    .withMotionMagic(MotorConfigs.geMotionMagicConfigs(AlgaeConstants.flywheelMMAcceleration,
    AlgaeConstants.flywheelMMCruiseVelocity, AlgaeConstants.flywheelMMJerk));
  
    TalonFXConfiguration innerRollerConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive));
    
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("KrakenX60"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive))
    .withMotionMagic(MotorConfigs.geMotionMagicConfigs(AlgaeConstants.pivotMMAcceleration,
    AlgaeConstants.pivotMMCruiseVelocity, AlgaeConstants.pivotMMJerk));
  
    m_leftFlywheel.getConfigurator().apply(flywheelConfigs);
    m_rightFlywheel.getConfigurator().apply(flywheelConfigs);
    m_innerRollers.getConfigurator().apply(innerRollerConfigs);
    m_leftPivot.getConfigurator().apply(pivotConfigs);
    m_rightPivot.getConfigurator().apply(pivotConfigs);
  }
  
  public boolean isPivotEncoderReset(){
    return isPivotEncoderReset;
  }
  
  public static boolean isAlgaeIntaked(){
    return algaeSensor.get();
}

private void setControl(TalonFX motor, ControlRequest req) {
  if (motor.isAlive() && isPivotEncoderReset) {
    motor.setControl(req);
  }
}

private void stopMotor(TalonFX motor) {
  motor.stopMotor();
}

private void zeroPivot(){ 
  m_leftPivot.setPosition(0).isOK();
  m_rightPivot.setPosition(0).isOK();
  currentState = AlgaeStates.PIVOT_DOCK_SHOOT;
}

public void setBothFlywheelVelocity(double flywheelVelocity){
  setControl(m_leftFlywheel, motionMagicVelocityVoltage.withVelocity(flywheelVelocity));
  setControl(m_rightFlywheel, motionMagicVelocityVoltage.withVelocity(flywheelVelocity));
}

public void setPivotPosition(double pivotPosition){
  setControl(m_leftPivot, motionMagicVoltage.withPosition(pivotPosition));
  setControl(m_rightPivot, motionMagicVoltage.withPosition(pivotPosition));
}

public void setInnerRollersVelocity(double rollerVoltage){
  setControl(m_innerRollers, voltageOut.withOutput(rollerVoltage));
}

public boolean arePivotsAtSetPoint(){
  return (Math.abs(m_leftPivot.getPosition().getValueAsDouble() -
    currentState.getSetpointValue()) < AlgaeConstants.pivotAllowedError) && (Math.abs(m_rightPivot.getPosition().getValueAsDouble() -
    currentState.getSetpointValue()) < AlgaeConstants.pivotAllowedError) ;
}

public void pivotTransitionHandler(AlgaeStates wantedState) {
    switch (wantedState) {
        case PIVOT_INTAKE, PIVOT_PROCESSOR, PIVOT_DOCK_SHOOT, PIVOT_LVL2REEF:
          setPivotPosition(wantedState.getSetpointValue());
        case FLYWHEEL_INTAKE, FLYWHEEL_SHOOT:
          setBothFlywheelVelocity(wantedState.getSetpointValue());
        case INNERROLLER_INTAKE, INNERROLLER_OUTTAKE:
          setInnerRollersVelocity(wantedState.getSetpointValue());
    }
    currentState = wantedState;
}


  @Override
  public void periodic() {
   if (DriverStation.isDisabled() && m_leftPivot.isAlive() && m_rightPivot.isAlive()){
      if (m_zeroSwitch.get()){
        zeroPivot();
      }
    }
  }

  public enum AlgaeStates{
    FLYWHEEL_INTAKE(AlgaeConstants.flywheelIntakeVelocity),
    FLYWHEEL_SHOOT(AlgaeConstants.flywheelShootVelocity),
    INNERROLLER_INTAKE(AlgaeConstants.innerRollersVoltage),
    INNERROLLER_OUTTAKE(-AlgaeConstants.innerRollersVoltage),
    PIVOT_INTAKE(AlgaeConstants.pivotIntakePos),
    PIVOT_PROCESSOR(AlgaeConstants.pivotProcessorPos),
    PIVOT_LVL2REEF(AlgaeConstants.pivotLvl2Pos),
    PIVOT_DOCK_SHOOT(AlgaeConstants.pivotShootingPos);

    private final double setpointValue;

    AlgaeStates(double setpointValue) {
      this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
      return setpointValue;
    }
  }
}
