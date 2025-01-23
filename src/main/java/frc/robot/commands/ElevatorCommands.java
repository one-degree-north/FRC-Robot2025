// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralMechCommands.CoralCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands extends Command {
  /** Creates a new ElevatorCommands. */
    private final Elevator elevator;
    private final ElevatorCommandsEnum commandType;
    private ElevatorStates wantedState;

  public ElevatorCommands(Elevator elevator, ElevatorCommandsEnum commandType) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.commandType = commandType;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (commandType) {
            case ELEVATOR_DOCKED:
                wantedState = ElevatorStates.ELEVATOR_DOCKED;
                break;
            case ELEVATOR_UP:
                wantedState = ElevatorStates.ELEVATOR_UP;
                break;
            case ELEVATOR_DOWN:
                wantedState = ElevatorStates.ELEVATOR_DOWN;
                break;
            case ELEVATOR_L1:
                wantedState = ElevatorStates.ELEVATOR_L1;
                break;
            case ELEVATOR_L2:
                wantedState = ElevatorStates.ELEVATOR_L2;
                break;
            case ELEVATOR_L3:
                wantedState = ElevatorStates.ELEVATOR_L3;
                break;
            case ELEVATOR_L4:
                wantedState = ElevatorStates.ELEVATOR_L4;
                break;
            default:
                return;
        }
        // Execute initial state transition
        Commands.run(() -> elevator.elevatorTransitionHandler(wantedState), elevator)
            .alongWith(Commands.waitUntil(elevator::isElevatorAtSetpoint)).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      elevator.setElevatorMotorsVoltage(0);
  }


  public enum ElevatorCommandsEnum {
    ELEVATOR_UP,
    ELEVATOR_DOWN,
    ELEVATOR_DOCKED,
    ELEVATOR_L1,
    ELEVATOR_L2,
    ELEVATOR_L3,
    ELEVATOR_L4,
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return switch (commandType) {
      case ELEVATOR_DOCKED, ELEVATOR_L1, ELEVATOR_L2, ELEVATOR_L3, ELEVATOR_L4 -> elevator.isElevatorAtSetpoint();
      default -> false;
  };
  }
}
