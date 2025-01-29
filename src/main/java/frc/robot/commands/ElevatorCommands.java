// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands extends Command {
  /** Creates a new ElevatorCommands. */
    private final Elevator elevator;
    private final ElevatorStates commandType;
    private Command commandToRun;

  public ElevatorCommands(Elevator elevator, ElevatorStates commandType) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.commandType = commandType;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandToRun = Commands.run(() -> elevator.elevatorTransitionHandler(commandType), elevator).alongWith(Commands.waitUntil(() -> elevator.isElevatorAtSetpoint()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandToRun.execute();
  }

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
    return commandToRun.isFinished();
  }
}
