// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
public class AlgaeMech extends SubsystemBase {
  private TalonFX m_flywheelMaster;
  private TalonFX m_flywheelSlave;
  private TalonFX m_innerRollers;
  private TalonFX m_pivotMaster;
  private TalonFX m_pivotSlave;

  private MotionMagicVelocityVoltage motionMagicVelocityVoltage 
  = new MotionMagicVelocityVoltage(0).withSlot(0);
  private VoltageOut voltageOut = new VoltageOut(0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private DigitalInput m_zeroSwitch;
  private boolean isPivotEncoderReset;
  private static DigitalInput algaeSensor;
  private boolean pivotZeroed = false;
  
    private AlgaeStates currentState;
  
    /** Creates a new AlgaeMech. */
    public AlgaeMech(DigitalInput zeroSwitch) {
      setName("AlgaeMech");
      m_flywheelMaster = new TalonFX(AlgaeConstants.leftFlywheelID, "rio");
      m_flywheelSlave = new TalonFX(AlgaeConstants.rightFlywheelID, "rio");
      m_innerRollers = new TalonFX(AlgaeConstants.innerRollersID, "rio");
      m_pivotMaster = new TalonFX(AlgaeConstants.leftPivotID, "rio");
      m_pivotSlave = new TalonFX(AlgaeConstants.rightPivotID, "rio");
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
  
    m_flywheelMaster.getConfigurator().apply(flywheelConfigs);
    m_flywheelSlave.getConfigurator().apply(flywheelConfigs);
    m_innerRollers.getConfigurator().apply(innerRollerConfigs);
    m_pivotMaster.getConfigurator().apply(pivotConfigs);
    m_pivotSlave.getConfigurator().apply(pivotConfigs);
    configureFollowers();
  }

  private void configureFollowers() {
    m_flywheelSlave.setControl(new Follower(m_flywheelMaster.getDeviceID(), false));
    m_pivotSlave.setControl(new Follower(m_pivotMaster.getDeviceID(), false));
  }

  public boolean isPivotEncoderReset(){
    return isPivotEncoderReset;
  }

  public boolean isAlgaeIntaked(){
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

  public void stopAll(){
    stopMotor(m_flywheelMaster);
    stopMotor(m_innerRollers);
    stopMotor(m_pivotMaster);
  }

  private void zeroPivot(){ 
    m_pivotMaster.setPosition(0);
    currentState = AlgaeStates.PIVOT_DOCK_SHOOT;
  }

  public void setBothFlywheelVelocity(double flywheelVelocity){
    setControl(m_flywheelMaster, motionMagicVelocityVoltage.withVelocity(flywheelVelocity));
  }

  public void setPivotPosition(double pivotPosition){
    setControl(m_pivotMaster, motionMagicVoltage.withPosition(pivotPosition));
  }

  public void setInnerRollersVelocity(double rollerVoltage){
    setControl(m_innerRollers, voltageOut.withOutput(rollerVoltage));
  }

  public boolean arePivotsAtSetPoint(){
    return (Math.abs(m_pivotMaster.getPosition().getValueAsDouble() -
      currentState.getSetpointValue()) < AlgaeConstants.pivotAllowedError);
  }

  public boolean isFlywheelAtTargetSpeed(){
    double currentVelocity = m_flywheelMaster.getVelocity().getValueAsDouble(); // double check gear ratios
    double targetVelocity = currentState.getSetpointValue(); // Ensure this returns the target velocity for the flywheel
    return Math.abs(currentVelocity - targetVelocity) < AlgaeConstants.flywheelAllowedError;
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

  private final SysIdRoutine flywheelCharacterization =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Voltage.ofBaseUnits(6, Volt),
            null
        ),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
                m_flywheelMaster.setControl(voltageOut.withOutput(volts));
            },
            null,  // No feedback mechanism needed for now
            this
        )
    );

  private final SysIdRoutine algaeRoutineToRun = pivotCharacterization;
  
  public Command algaeSysIdDynamic(SysIdRoutine.Direction direction) {
    return algaeRoutineToRun.dynamic(direction);
  }

  public Command algaeSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return algaeRoutineToRun.quasistatic(direction);
  }

  public void pivotTransitionHandler(AlgaeStates wantedState) {
      switch (wantedState) {
          case PIVOT_INTAKE, PIVOT_PROCESSOR, PIVOT_DOCK_SHOOT, PIVOT_LVL2REEF:
            setPivotPosition(wantedState.getSetpointValue());
            break;
          case FLYWHEEL_INTAKE, FLYWHEEL_SHOOT:
            setBothFlywheelVelocity(wantedState.getSetpointValue());
            break;
          case INNERROLLER_INTAKE, INNERROLLER_OUTTAKE:
            setInnerRollersVelocity(wantedState.getSetpointValue());
            break;
      }
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

    SmartDashboard.putString("AlgaeState", (currentState != null) ? currentState.name() : "UNKNOWN");
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
