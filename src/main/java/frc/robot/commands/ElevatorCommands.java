package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class ElevatorCommands extends Command {
  /** Creates a new ElevatorCommands. */
    private final Elevator elevator;
    private final ElevatorStates commandType;
    private Command commandToRun;

  public ElevatorCommands(Elevator elevator, ElevatorStates commandType) {
    this.elevator = elevator;
    this.commandType = commandType;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    commandToRun = Commands.sequence( 
        Commands.run(() -> elevator.elevatorTransitionHandler(commandType), elevator),
        Commands.waitUntil(() -> elevator.isElevatorAtSetpoint())
    );
  }

  @Override
  public void execute() {
    commandToRun.execute();
  }

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

  @Override
  public boolean isFinished() {
    return commandToRun.isFinished();
  }
}
