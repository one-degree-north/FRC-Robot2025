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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.AlgaeMechCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.CoralMechCommands;
import frc.robot.commands.ElevatorCommands;

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
    public final CoralMech s_CoralMech = new CoralMech(zeroSwitch);
    public final AlgaeMech s_AlgaeMech = new AlgaeMech(zeroSwitch);
    public final Climb s_Climb = new Climb();
    public final Elevator s_Elevator = new Elevator(zeroSwitch);

    @Logged
    private RobotMode currentMode = RobotMode.DRIVING; // Default mode
    private SUBSYSTEMTOTUNE currentTuneSubsystem = SUBSYSTEMTOTUNE.DRIVETRAIN; // Default subsystem

    public RobotContainer() {
        SmartDashboard.putBoolean("Tuning Mode", false);
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
        //arbitrary bindings, change later depending on driver preferences
        joystick.square().onTrue(new AlgaeMechCommands(s_AlgaeMech, AlgaeMechCommands.AlgaeCommands.GROUNDINTAKE));
        joystick.cross().onTrue(new AlgaeMechCommands(s_AlgaeMech, AlgaeMechCommands.AlgaeCommands.REEFINTAKE));

        joystick.L1().onTrue(new ClimbCommands(s_Climb, Climb.ClimbStates.CLIMBUP));
        joystick.R1().onTrue(new ClimbCommands(s_Climb, Climb.ClimbStates.CLIMBDOWN));

        joystick.triangle().onTrue(new CoralMechCommands(s_CoralMech, CoralMechCommands.CoralCommands.HPINTAKE));
        joystick.circle().onTrue(new CoralMechCommands(s_CoralMech, CoralMechCommands.CoralCommands.REEFOUTTAKE));

        joystick.L2().onTrue(new ElevatorCommands(s_Elevator, Elevator.ElevatorStates.ELEVATOR_UP));
        joystick.R2().onTrue(new ElevatorCommands(s_Elevator, Elevator.ElevatorStates.ELEVATOR_DOWN));
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
                joystick.R1().and(joystick.square()).whileTrue(s_AlgaeMech.algaeSysIdDynamic(Direction.kForward));
                joystick.R1().and(joystick.triangle()).whileTrue(s_AlgaeMech.algaeSysIdDynamic(Direction.kReverse));
                joystick.R2().and(joystick.square()).whileTrue(s_AlgaeMech.algaeSysIdQuasistatic(Direction.kForward));
                joystick.R2().and(joystick.triangle()).whileTrue(s_AlgaeMech.algaeSysIdQuasistatic(Direction.kReverse));
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

    private enum RobotMode {
        DRIVING, TUNING
    }

    // Add more for every subsystem
    private enum SUBSYSTEMTOTUNE {
        DRIVETRAIN, CORALMECH, ALGAEMECH, ELEVATOR
    }
}
