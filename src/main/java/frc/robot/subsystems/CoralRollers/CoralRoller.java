// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralRollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.CoralConstants;

public class CoralRoller extends SubsystemBase {
  private TalonFX m_leftRoller;
  private TalonFX m_rightRoller;
  private VoltageOut voltageOut = new VoltageOut(0);

  public static boolean isCoralIntaked = false;

  public CoralRoller() {
    m_leftRoller = new TalonFX(CoralConstants.leftRollerID, "rio");
    m_rightRoller = new TalonFX(CoralConstants.rightRollerID, "rio");
    configureMotors();
  }

  private void configureMotors(){
    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Coast, InvertedValue.Clockwise_Positive));

    m_leftRoller.getConfigurator().apply(rollerConfigs);
    m_rightRoller.getConfigurator().apply(rollerConfigs);
}

private void setControl(TalonFX motor, ControlRequest req) {
  if (motor.isAlive()) {
    motor.setControl(req);
  }
}

private void stopMotor(TalonFX motor) {
  motor.stopMotor();
}

public void setBothRollersVoltage(double voltage) {
  setControl(m_leftRoller, voltageOut.withOutput(voltage));
  setControl(m_rightRoller, voltageOut.withOutput(voltage));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum CoralStates {
    ROLLER_INTAKE(CoralConstants.rollerIntakeVoltage),
    ROLLER_OUTTAKE(CoralConstants.rollerOuttakeVoltage),
    ROLLER_L1OUTAKE(CoralConstants.rollerOuttakeVoltage);

    private final double setpointValue;

    CoralStates(double setpointValue) {
      this.setpointValue = setpointValue;
    }

    public double getSetpointValue() {
      return setpointValue;
    }
  }
}
