// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRollers;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.AlgaeConstants;

public class AlgaeFlywheel extends SubsystemBase {
  private TalonFX m_flywheelMaster;
  private TalonFX m_flywheelSlave;

  private MotionMagicVelocityVoltage motionMagicVelocityVoltage 
  = new MotionMagicVelocityVoltage(0).withSlot(0);
  private VoltageOut voltageOut = new VoltageOut(0);

  private FlywheelStates currentState;
  private FlywheelStates desiredState;

  public AlgaeFlywheel() {
    m_flywheelMaster = new TalonFX(AlgaeConstants.flywheelMasterID, "rio");
    m_flywheelSlave = new TalonFX(AlgaeConstants.flywheelSlaveID, "rio");
    configureMotors();
  }

  private void configureMotors(){
    TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive))
    .withFeedback(MotorConfigs.getFeedbackConfigs(AlgaeConstants.flywheelMechanismRatio))
    .withMotionMagic(MotorConfigs.geMotionMagicConfigs(AlgaeConstants.flywheelMMAcceleration,
    AlgaeConstants.flywheelMMCruiseVelocity, AlgaeConstants.flywheelMMJerk))
    .withSlot0(MotorConfigs.getSlot0Configs(AlgaeConstants.flywheelkP, AlgaeConstants.flywheelkI,
    AlgaeConstants.flywheelkD, AlgaeConstants.flywheelkS, AlgaeConstants.flywheelkV, AlgaeConstants.flywheelkA, AlgaeConstants.flywheelkG));

    m_flywheelMaster.getConfigurator().apply(flywheelConfigs);
    m_flywheelSlave.getConfigurator().apply(flywheelConfigs);
    configureFollower();
  }

  private void configureFollower(){
    m_flywheelSlave.setControl(new Follower(m_flywheelMaster.getDeviceID(), false));
  }

  private void setControl(TalonFX motor, ControlRequest req) {
    if (motor.isAlive()) {
      motor.setControl(req);
    }
  }

  private void stopMotor(TalonFX motor) {
    motor.stopMotor();
  }

  public void setBothFlywheelVelocity(double flywheelVelocity){
    setControl(m_flywheelMaster, motionMagicVelocityVoltage.withVelocity(flywheelVelocity));
  }

  public boolean isFlywheelAtTargetSpeed(){
    double currentVelocity = m_flywheelMaster.getVelocity().getValueAsDouble(); // double check gear ratios
    double targetVelocity = currentState.getSetpointValue(); // Ensure this returns the target velocity for the flywheel
    return Math.abs(currentVelocity - targetVelocity) < AlgaeConstants.flywheelAllowedError;
  }

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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum FlywheelStates{
    FLYWHEEL_SHOOT(AlgaeConstants.flywheelShootVelocity),
    FLYWHEEL_INTAKE(AlgaeConstants.flywheelIntakeVelocity);

    private final double setpointValue;

    FlywheelStates(double setpointValue) {
      this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
      return setpointValue;
    }
  }
}
