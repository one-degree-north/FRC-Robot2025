// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.AlgaeConstants;

public class AlgaeInnerRollers extends SubsystemBase {
  private TalonFX m_innerRollers;
  private static DigitalInput algaeSensor;
  private VoltageOut voltageOut = new VoltageOut(0);

  public AlgaeInnerRollers() {
    m_innerRollers = new TalonFX(AlgaeConstants.innerRollersID, "rio");
    algaeSensor = new DigitalInput(AlgaeConstants.beamBreakID);
    configureMotors();
  }

  private void configureMotors(){
    TalonFXConfiguration innerRollerConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, InvertedValue.Clockwise_Positive));

      m_innerRollers.getConfigurator().apply(innerRollerConfigs);
  }

  public static boolean isAlgaeIntaked(){
    return algaeSensor.get();
  }

  private void setControl(TalonFX motor, ControlRequest req) {
    if (motor.isAlive()) {
      motor.setControl(req);
    }
  }

  private void stopMotor(TalonFX motor) {
    motor.stopMotor();
  }

  public void setInnerRollersVelocity(double rollerVoltage){
    setControl(m_innerRollers, voltageOut.withOutput(rollerVoltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
