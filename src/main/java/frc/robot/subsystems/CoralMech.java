// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.CoralConstants;

public class CoralMech extends SubsystemBase {
  private TalonFX m_leftRoller;
  private TalonFX m_rightRoller;
  private TalonFX m_krakenWrist;

  private VoltageOut voltageOut = new VoltageOut(0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private CoralStates currentState;
  private DigitalInput m_zeroSwitch;

  private boolean isWristEncoderReset;

  /** Creates a new CoralMech. */
  public CoralMech(DigitalInput zeroSwitch) {
    setName("CoralMech");
    m_leftRoller = new TalonFX(CoralConstants.leftRollerID, "rio");
    m_rightRoller = new TalonFX(CoralConstants.rightRollerID, "rio");
    m_krakenWrist = new TalonFX(CoralConstants.wristRollerID, "rio");
    m_zeroSwitch = zeroSwitch;
    motorConfigurations();
  }

  private void motorConfigurations(){
    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Coast, InvertedValue.Clockwise_Positive));
    
    TalonFXConfiguration wristConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("KrakenX60"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive))
    .withSlot0(MotorConfigs.getSlot0Configs(
      CoralConstants.wristkP, CoralConstants.wristkI, CoralConstants.wristkD, CoralConstants.wristkS,
      CoralConstants.wristkV, CoralConstants.wristkA,0))
    .withFeedback(MotorConfigs.getFeedbackConfigs(CoralConstants.wristMechanismRatio))
    .withMotionMagic(MotorConfigs.geMotionMagicConfigs(CoralConstants.wristMMAcceleration,
     CoralConstants.wristMMCruiseVelocity, CoralConstants.wristMMJerk));
    
    m_leftRoller.getConfigurator().apply(rollerConfigs);
    m_rightRoller.getConfigurator().apply(rollerConfigs);
    m_krakenWrist.getConfigurator().apply(wristConfigs);
  }

public boolean isWristEncoderReset(){
  return isWristEncoderReset;
}

private void setControl(TalonFX motor, ControlRequest req) {
  if (motor.isAlive() && isWristEncoderReset) {
    motor.setControl(req);
  }
}

public void setBothRollersVoltage(double voltage) {
  setControl(m_leftRoller, voltageOut.withOutput(voltage));
  setControl(m_rightRoller, voltageOut.withOutput(voltage));
}

private void stopMotor(TalonFX motor) {
  motor.stopMotor();
}

private void zeroWrist(){
  m_krakenWrist.setPosition(0).isOK();
  currentState = CoralStates.WRIST_DOCKED;
}

private double getRollerVelocity(TalonFX motor) {
  return motor.getVelocity().getValueAsDouble();
}

public Boolean isWristAtSetpoint() {
  return Math.abs(m_krakenWrist.getPosition().getValueAsDouble() -
    currentState.getSetpointValue()) < CoralConstants.wristAllowedError;
}

public void coralTransitionHandler(CoralStates wantedState) {
    switch (wantedState) {
        case ROLLER_INTAKE, ROLLER_OUTTAKE -> 
          setBothRollersVoltage(wantedState.getSetpointValue());
        case ROLLER_L1OUTAKE -> {
          setControl(m_leftRoller, voltageOut.withOutput(CoralStates.ROLLER_OUTTAKE.getSetpointValue()));
          stopMotor(m_rightRoller);
        }
        case WRIST_REEF, WRIST_HP, WRIST_DOCKED -> 
          setControl(m_krakenWrist, motionMagicVoltage.withPosition(wantedState.getSetpointValue()));
    }
    currentState = wantedState;
}

private final SysIdRoutine coralWristCharacterization =
  new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism((Voltage volts) -> m_krakenWrist.setControl(voltageOut.withOutput(volts)), 
    null, 
    this)
  );


  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && m_krakenWrist.isAlive()){
      if (m_zeroSwitch.get()){
        zeroWrist();
      }
    }
    SmartDashboard.putBoolean("IsWristEncoderReset?", isWristEncoderReset);
    SmartDashboard.putBoolean("IsWristAtSetpoint?", isWristAtSetpoint());
    SmartDashboard.putNumber("LeftRollerVelocity", getRollerVelocity(m_leftRoller));
    SmartDashboard.putNumber("RightRollerVelocity", getRollerVelocity(m_rightRoller));
    SmartDashboard.putString("CoralState", currentState.name());
  }
  
  public enum CoralStates {
    ROLLER_INTAKE(CoralConstants.rollerIntakeVoltage),
    ROLLER_OUTTAKE(CoralConstants.rollerOuttakeVoltage),
    ROLLER_L1OUTAKE(CoralConstants.rollerOuttakeVoltage),
    WRIST_REEF(CoralConstants.wristReefPos),
    WRIST_HP(CoralConstants.wristIntakePos),
    WRIST_DOCKED(CoralConstants.wristDockedPos);

    private final double setpointValue;

    CoralStates(double setpointValue) {
      this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
      return setpointValue;
    }
}
}
