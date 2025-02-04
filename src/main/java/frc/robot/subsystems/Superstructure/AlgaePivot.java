// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
import frc.robot.constants.RegularConstants.AlgaeConstants;

@Logged
public class AlgaePivot extends SubsystemBase {
  private TalonFX m_pivotMaster;
  private TalonFX m_pivotSlave;

  private MotionMagicVelocityVoltage motionMagicVelocityVoltage 
  = new MotionMagicVelocityVoltage(0).withSlot(0);
  private VoltageOut voltageOut = new VoltageOut(0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private DigitalInput m_zeroSwitch;
  private boolean isPivotEncoderReset;
  private boolean pivotZeroed = false;
  
  private AlgaeStates currentState;
  
    /** Creates a new AlgaeMech. */
    public AlgaePivot(DigitalInput zeroSwitch) {
      setName("AlgaeMech");
      m_pivotMaster = new TalonFX(AlgaeConstants.pivotMasterID, "rio");
      m_pivotSlave = new TalonFX(AlgaeConstants.pivotSlaveID, "rio");
      m_zeroSwitch = zeroSwitch;
      configMotors();
    }
  
  private void configMotors(){    
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("KrakenX60"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive))
    .withMotionMagic(MotorConfigs.geMotionMagicConfigs(AlgaeConstants.pivotMMAcceleration,
    AlgaeConstants.pivotMMCruiseVelocity, AlgaeConstants.pivotMMJerk))
    .withSlot0(MotorConfigs.getSlot0Configs(AlgaeConstants.pivotkP, AlgaeConstants.pivotkI,
    AlgaeConstants.pivotkD, AlgaeConstants.pivotkS, AlgaeConstants.pivotkV, AlgaeConstants.pivotkA, AlgaeConstants.pivotkG));
  
    m_pivotMaster.getConfigurator().apply(pivotConfigs);
    m_pivotSlave.getConfigurator().apply(pivotConfigs);
    configureFollowers();
  }

  private void configureFollowers() {
    m_pivotSlave.setControl(new Follower(m_pivotMaster.getDeviceID(), false));
  }

  public boolean isPivotEncoderReset(){
    return isPivotEncoderReset;
  }

  private void setControl(TalonFX motor, ControlRequest req) {
    if (motor.isAlive() && isPivotEncoderReset) {
      motor.setControl(req);
    }
  }

  private void stopMotor(TalonFX motor) {
    motor.stopMotor();
  }

  public void stopAll(){
    stopMotor(m_pivotMaster);
  }

  private void zeroPivot(){ 
    m_pivotMaster.setPosition(0);
    currentState = AlgaeStates.PIVOT_DOCK_SHOOT;
  }

  public void setPivotPosition(double pivotPosition){
    setControl(m_pivotMaster, motionMagicVoltage.withPosition(pivotPosition));
  }

  public boolean arePivotsAtSetPoint(){
    return (Math.abs(m_pivotMaster.getPosition().getValueAsDouble() -
      currentState.getSetpointValue()) < AlgaeConstants.pivotAllowedError);
  }

  private final SysIdRoutine pivotCharacterization =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Voltage.ofBaseUnits(3, Volt),
              null
          ),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> {
                  m_pivotMaster.setControl(voltageOut.withOutput(volts));
              },
              null,  // No feedback mechanism needed for now
              this
          )
      );

  
  public Command algaePivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotCharacterization.dynamic(direction);
  }

  public Command algaePivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotCharacterization.quasistatic(direction);
  }

  public void pivotTransitionHandler(AlgaeStates wantedState) {
    setPivotPosition(wantedState.getSetpointValue());
    currentState = wantedState;
  }

  @Override
  public void periodic() {
   if (DriverStation.isDisabled() && !pivotZeroed){
      if (m_zeroSwitch.get()){
        zeroPivot();
        pivotZeroed = true;
      }
      else if (!DriverStation.isDisabled()){
        pivotZeroed = false;
      }
    }

    SmartDashboard.putString("AlgaePivotState", (currentState != null) ? currentState.name() : "UNKNOWN");
  }

  public enum AlgaeStates{
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
