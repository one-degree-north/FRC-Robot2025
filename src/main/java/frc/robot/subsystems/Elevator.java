package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.MotorConfigs;
import frc.robot.constants.RegularConstants.ElevatorConstants;

@Logged
public class Elevator extends SubsystemBase {
  private TalonFX m_elevatorMasterMotor;
  private TalonFX m_elevatorSlaveMotor;
  private DutyCycleEncoder m_elevatorEncoder;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput m_zeroSwitch;

  private VoltageOut voltageOut = new VoltageOut(0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  
  private ElevatorStates currentState = ElevatorStates.ELEVATOR_DOCKED; // Initialize to a safe state
  private boolean isElevatorEncoderReset = false;
  private boolean hasResetOccurred = false; // Prevent repeated resets

  public Elevator(DigitalInput zeroSwitch) {
    setName("Elevator");
    m_elevatorMasterMotor = new TalonFX(ElevatorConstants.leftElevatorID, "rio");
    m_elevatorSlaveMotor = new TalonFX(ElevatorConstants.rightElevatorID, "rio");
    m_elevatorEncoder = new DutyCycleEncoder(ElevatorConstants.elevatorEncoderID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.magneticLimitSwitchID);
    m_zeroSwitch = zeroSwitch;
    motorConfigurations();
  }


  private void motorConfigurations(){
    m_elevatorMasterMotor.getConfigurator().apply(motorConfigurationInverted(InvertedValue.Clockwise_Positive));
    m_elevatorSlaveMotor.getConfigurator().apply(motorConfigurationInverted(InvertedValue.CounterClockwise_Positive));

    // Set the follower motor to follow the master motor
    Follower followerConfig = new Follower(m_elevatorMasterMotor.getDeviceID(), true);
    m_elevatorSlaveMotor.setControl(followerConfig);
  }

  private TalonFXConfiguration motorConfigurationInverted(InvertedValue invertedValue){
    TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("KrakenX60"))
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Brake, invertedValue))
    .withSlot0(MotorConfigs.getSlot0Configs(
      ElevatorConstants.ElevatorkP, ElevatorConstants.ElevatorkI, ElevatorConstants.ElevatorkD, 
      ElevatorConstants.ElevatorkS, ElevatorConstants.ElevatorkV, ElevatorConstants.ElevatorkA, ElevatorConstants.ElevatorkG))
    .withFeedback(MotorConfigs.getFeedbackConfigs(ElevatorConstants.ElevatorMechanismRatio))
    .withMotionMagic(MotorConfigs.geMotionMagicConfigs(ElevatorConstants.ElevatorMMAcceleration,
     ElevatorConstants.ElevatorMMCruiseVelocity, ElevatorConstants.ElevatorMMJerk));
     return elevatorConfigs;
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
    setControl(m_elevatorMasterMotor, voltageOut.withOutput(voltage));
  }

  private final SysIdRoutine elevatorCharacterization =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Voltage.ofBaseUnits(3, Volt),
            null
        ),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> {
                m_elevatorMasterMotor.setControl(voltageOut.withOutput(volts));
            },
            null,
            this
        )
    );

  public Command elevatorSysIDQuasistatic(SysIdRoutine.Direction direction){
    return elevatorCharacterization.quasistatic(direction);
  }

  public Command elevatorSysIDDynamic(SysIdRoutine.Direction direction){
    return elevatorCharacterization.dynamic(direction);
  }

  public boolean isElevatorAtSetpoint() {
    return Math.abs(m_elevatorEncoder.get() - currentState.getSetpointValue()) < ElevatorConstants.wristAllowedError;
  }

  public void elevatorTransitionHandler(ElevatorStates wantedState) {
    switch (wantedState) {
        case ELEVATOR_UP, ELEVATOR_DOWN:
          setElevatorMotorsVoltage(wantedState.getSetpointValue());
          break;
        case ELEVATOR_DOCKED, ELEVATOR_L1, ELEVATOR_L2, ELEVATOR_L3, ELEVATOR_L4:
          setControl(m_elevatorMasterMotor, motionMagicVoltage.withPosition(wantedState.getSetpointValue()));
          setControl(m_elevatorSlaveMotor, motionMagicVoltage.withPosition(wantedState.getSetpointValue()));
          break;
    }
    currentState = wantedState;
  }

  public enum ElevatorStates {
    ELEVATOR_UP(ElevatorConstants.elevatorUpVoltage),
    ELEVATOR_DOWN(-ElevatorConstants.elevatorUpVoltage),
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
    // Reset encoder ONCE if both limit switches are active AND we haven't reset yet
    if (!hasResetOccurred && DriverStation.isDisabled() && bottomLimitSwitch.get() && m_zeroSwitch.get()) {
        currentState = ElevatorStates.ELEVATOR_DOCKED;
        m_elevatorMasterMotor.setPosition(0); // Reset encoder position to zero
        m_elevatorSlaveMotor.setPosition(0);
        isElevatorEncoderReset = true;
        hasResetOccurred = true; // Prevent repeated resets
    }

    // Allow reset again once the elevator moves off the switch
    if (!bottomLimitSwitch.get() || !m_zeroSwitch.get()) {
        hasResetOccurred = false;
    }

    // Safely log state
    SmartDashboard.putString("ElevatorState", (currentState != null) ? currentState.name() : "UNKNOWN");
  }
}
