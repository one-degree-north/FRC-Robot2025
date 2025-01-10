package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.CoralMech.CoralStates;

public class CoralMechCommands extends Command {
    private final CoralMech coralMech;
    private final CoralCommands commandType;
    private CoralStates initialState;
    private CoralStates finalState;

    public CoralMechCommands(CoralMech coralMech, CoralCommands commandType) {
        this.coralMech = coralMech;
        this.commandType = commandType;
        addRequirements(coralMech);
    }

    @Override
    public void initialize() {
        switch (commandType) {
            case DOCKED:
                initialState = CoralStates.WRIST_DOCKED;
                break;
            case HPINTAKE:
                initialState = CoralStates.WRIST_HP;
                finalState = CoralStates.ROLLER_INTAKE;
                break;
            case REEFOUTTAKE:
                initialState = CoralStates.WRIST_REEF;
                finalState = CoralStates.ROLLER_OUTTAKE;
                break;
            case LVL1OUTTAKE:
                initialState = CoralStates.WRIST_REEF;
                finalState = CoralStates.ROLLER_L1OUTAKE;
                break;
            default:
                return;
        }
        // Execute initial state transition
        Commands.run(() -> coralMech.coralStateTransition(initialState), coralMech)
            .alongWith(Commands.waitUntil(coralMech::isWristAtSetpoint))
            .andThen(() -> {
              if (finalState != null) { //for DOCKED case finalState is null
                  coralMech.coralStateTransition(finalState);
              }
          })
            .schedule();
    }

    @Override
    public boolean isFinished() {
        return switch (commandType) {
            case DOCKED, REEFOUTTAKE, LVL1OUTTAKE -> coralMech.isWristAtSetpoint();
            default -> false;
        };
    }

    @Override
    public void end(boolean interrupted) {
        if (commandType != CoralCommands.DOCKED) {
            coralMech.setBothRollersVoltage(0);
        }
    }

    public enum CoralCommands {
        DOCKED,
        HPINTAKE,
        REEFOUTTAKE,
        LVL1OUTTAKE,
    }
}
