// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.CoralConstants;

@Logged
public class CoralWrist extends SubsystemBase {
  private TalonFX m_krakenWrist;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private VoltageOut voltageOut = new VoltageOut(0);
  private CoralStates currentState;
  private CoralStates desiredState;
  private DigitalInput m_zeroSwitch;

  private boolean isWristEncoderReset;

  /** Creates a new CoralMech. */
  public CoralWrist(DigitalInput zeroSwitch) {
    setName("CoralMech");
    m_krakenWrist = new TalonFX(CoralConstants.wristRollerID, "rio");
    m_zeroSwitch = zeroSwitch;
    configureMotors();
  }

  private void configureMotors(){    
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

private void stopMotor(TalonFX motor) {
  motor.stopMotor();
}

public void stopAllMotors(){
  stopMotor(m_krakenWrist);
}

private void zeroWrist(){
  m_krakenWrist.setPosition(0);
  currentState = CoralStates.WRIST_DOCKED;
}

public Boolean isWristAtSetpoint() {
  return Math.abs(m_krakenWrist.getPosition().getValueAsDouble() -
    currentState.getSetpointValue()) < CoralConstants.wristAllowedError;
}

private void setDesiredState(CoralStates state) {
  desiredState = state;
}

public void coralTransitionHandler(CoralStates desiredState) {
  setControl(m_krakenWrist, motionMagicVoltage.withPosition(desiredState.getSetpointValue()));
  currentState = desiredState;
}

private final SysIdRoutine coralWristCharacterization = 
  new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Voltage.ofBaseUnits(3, Volt),
      null
    ),
    new SysIdRoutine.Mechanism(
      (Voltage volts) -> m_krakenWrist.setControl(voltageOut.withOutput(volts)), 
  null, 
      this)
  );

public Command coralSysIDQuasistatic(SysIdRoutine.Direction direction){
  return coralWristCharacterization.quasistatic(direction);
}

public Command coralSysIDDynamic(SysIdRoutine.Direction direction){
  return coralWristCharacterization.dynamic(direction);
}


  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && m_krakenWrist.isAlive()){
      if (m_zeroSwitch.get()){
        zeroWrist();
      }
    }

    if (desiredState != currentState) {
      coralTransitionHandler(desiredState);
    }

    SmartDashboard.putBoolean("IsWristEncoderReset?", isWristEncoderReset);
    SmartDashboard.putBoolean("IsWristAtSetpoint?", isWristAtSetpoint());
    SmartDashboard.putString("CoralState", (currentState != null) ? currentState.name() : "UNKNOWN");
  }
  
  public enum CoralStates {
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
