package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.RegularConstants.MiscConstants;
import frc.robot.subsystems.Drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure.Elevator;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.Superstructure.AlgaePivot;
import frc.robot.subsystems.Superstructure.Climb;
import frc.robot.subsystems.Superstructure.CoralWrist;

public class RobotContainer {
    private DigitalInput zeroSwitch = new DigitalInput(MiscConstants.zeroSwitchID);

    @Logged
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
    private final CommandPS5Controller joystick = new CommandPS5Controller(MiscConstants.controllerID);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralWrist s_CoralMech = new CoralWrist(zeroSwitch);
    public final AlgaePivot s_AlgaeMech = new AlgaePivot(zeroSwitch);
    public final Climb s_Climb = new Climb();
    public final Elevator s_Elevator = new Elevator(zeroSwitch);
    public final LEDs s_LEDs = new LEDs(s_AlgaeMech);

    private RobotMode currentMode = RobotMode.DRIVING; // Default mode
    private SUBSYSTEMTOTUNE currentTuneSubsystem = SUBSYSTEMTOTUNE.DRIVETRAIN; // Default subsystem

    public RobotContainer() {
        SmartDashboard.putString("RobotMode", currentMode.name());
        configureBindings();
    }

    private void configureBindings() {
        // Default Drivetrain Command (applies in both modes)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // **Toggle between TUNING and DRIVING mode using touchpad**
        joystick.touchpad().onTrue(new InstantCommand(() -> {
            toggleMode();
            SmartDashboard.putBoolean("Tuning Mode", currentMode == RobotMode.TUNING);
        }));

        switch (currentMode) {
            case DRIVING:
                drivingBindings();
                break;
            case TUNING:
                tuningBindings();
                break;
        }
    }

    private void toggleMode() {
        currentMode = (currentMode == RobotMode.DRIVING) ? RobotMode.TUNING : RobotMode.DRIVING;
    }

    private void drivingBindings(){
        //configure later
    }

    private void tuningBindings() {
        switch(currentTuneSubsystem){
            case DRIVETRAIN:
                // Dynamic tuning for drivetrain
                joystick.R1().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.R1().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.R2().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.R2().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
                break;
    
            case CORALMECH:
                // SysID tuning for CoralMech
                joystick.R1().and(joystick.square()).whileTrue(s_CoralMech.coralSysIDDynamic(Direction.kForward));
                joystick.R1().and(joystick.triangle()).whileTrue(s_CoralMech.coralSysIDDynamic(Direction.kReverse));
                joystick.R2().and(joystick.square()).whileTrue(s_CoralMech.coralSysIDQuasistatic(Direction.kForward));
                joystick.R2().and(joystick.triangle()).whileTrue(s_CoralMech.coralSysIDQuasistatic(Direction.kReverse));
                break;
    
            case ALGAEMECH:
                // SysID tuning for AlgaeMech Pivot and Flywheel
                joystick.R1().and(joystick.square()).whileTrue(s_AlgaeMech.algaePivotSysIdDynamic(Direction.kForward));
                joystick.R1().and(joystick.triangle()).whileTrue(s_AlgaeMech.algaePivotSysIdDynamic(Direction.kReverse));
                joystick.R2().and(joystick.square()).whileTrue(s_AlgaeMech.algaePivotSysIdQuasistatic(Direction.kForward));
                joystick.R2().and(joystick.triangle()).whileTrue(s_AlgaeMech.algaePivotSysIdQuasistatic(Direction.kReverse));
                break;
    
            case ELEVATOR:
                // SysID tuning for Elevator
                joystick.R1().and(joystick.square()).whileTrue(s_Elevator.elevatorSysIDDynamic(Direction.kForward));
                joystick.R1().and(joystick.triangle()).whileTrue(s_Elevator.elevatorSysIDDynamic(Direction.kReverse));
                joystick.R2().and(joystick.square()).whileTrue(s_Elevator.elevatorSysIDQuasistatic(Direction.kForward));
                joystick.R2().and(joystick.triangle()).whileTrue(s_Elevator.elevatorSysIDQuasistatic(Direction.kReverse));
                break;
        }
    }
    

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public enum RobotMode {
        DRIVING, TUNING
    }

    // Add more for every subsystem
    private enum SUBSYSTEMTOTUNE {
        DRIVETRAIN, CORALMECH, ALGAEMECH, ELEVATOR
    }
}
