package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.AlgaeMech.AlgaeStates;

public class AlgaeMechCommands extends Command {
    private final AlgaeMech algaeMech;
    private final AlgaeCommands commandType;
    private AlgaeStates initialState;
    private AlgaeStates finalState;

    public AlgaeMechCommands(AlgaeMech algaeMech, AlgaeCommands commandType) {
        this.algaeMech = algaeMech;
        this.commandType = commandType;
        addRequirements(algaeMech);
    }

    @Override
    public void initialize() {
        switch (commandType) {
            case DOCKED:
                initialState = AlgaeStates.PIVOT_DOCK_SHOOT;
                break;
            case HPINTAKE:
                initialState = AlgaeStates.PIVOT_INTAKE;
                finalState = AlgaeStates.INNERROLLER_INTAKE;
                break;
            case REEFOUTTAKE:
                initialState = AlgaeStates.PIVOT_LVL2REEF;
                finalState = AlgaeStates.INNERROLLER_OUTTAKE;
                break;
            case LVL1OUTTAKE:
                initialState = AlgaeStates.PIVOT_PROCESSOR;
                finalState = AlgaeStates.INNERROLLER_OUTTAKE;
                break;
            default:
                return;
        }

        // Execute initial state transition
        Commands.run(() -> algaeMech.pivotTransitionHandler(initialState), algaeMech)
            .alongWith(Commands.waitUntil(algaeMech::arePivotsAtSetPoint))
            .andThen(() -> {
                if (finalState != null) { // for DOCKED case finalState is null
                    algaeMech.pivotTransitionHandler(finalState);
                }
            })
            .schedule();
    }

    @Override
    public boolean isFinished() {
        return switch (commandType) {
            case DOCKED, REEFOUTTAKE, LVL1OUTTAKE -> algaeMech.arePivotsAtSetPoint();
            default -> false;
        };
    }

    @Override
    public void end(boolean interrupted) {
        if (commandType != AlgaeCommands.DOCKED) {
            algaeMech.setInnerRollersVelocity(0);
            algaeMech.setBothFlywheelVelocity(0);
        }
    }

    public enum AlgaeCommands {
        DOCKED,
        HPINTAKE,
        REEFOUTTAKE,
        LVL1OUTTAKE,
    }
}
