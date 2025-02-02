package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.AlgaeMech.AlgaeStates;

public class AlgaeMechCommands extends Command {
    private final AlgaeMech s_AlgaeMech;
    private final AlgaeCommands commandType;
    private AlgaeStates initialState;
    private AlgaeStates finalState;
    private SequentialCommandGroup commandGroup;

    public AlgaeMechCommands(AlgaeMech algaeMech, AlgaeCommands commandType) {
        this.s_AlgaeMech = algaeMech;
        this.commandType = commandType;
        addRequirements(algaeMech);
    }

    @Override
    public void initialize() {
        // Determine initial and final states based on the command type
        switch (commandType) {
            case DOCKED:
                initialState = AlgaeStates.PIVOT_DOCK_SHOOT;
                break;
            case GROUNDINTAKE:
                initialState = AlgaeStates.PIVOT_INTAKE;
                finalState = AlgaeStates.INNERROLLER_INTAKE;
                break;
            case REEFINTAKE:
                initialState = AlgaeStates.PIVOT_LVL2REEF;
                finalState = AlgaeStates.INNERROLLER_INTAKE;
                break;
            case REEFOUTTAKE:
                initialState = AlgaeStates.PIVOT_LVL2REEF;
                finalState = AlgaeStates.INNERROLLER_OUTTAKE;
                break;
            case PROCESSOROUTTAKE:
                initialState = AlgaeStates.PIVOT_PROCESSOR;
                finalState = AlgaeStates.INNERROLLER_OUTTAKE;
                break;
            case SHOOTNET:
                initialState = AlgaeStates.PIVOT_DOCK_SHOOT;
                finalState = AlgaeStates.FLYWHEEL_SHOOT;
                break;
            default:
                return;
        }

        commandGroup = new SequentialCommandGroup(
            new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(initialState), s_AlgaeMech),
            new WaitUntilCommand(s_AlgaeMech::arePivotsAtSetPoint),
            new InstantCommand(() -> {
                if (finalState != null) {
                    s_AlgaeMech.pivotTransitionHandler(finalState);
                }
            }, s_AlgaeMech)
        );
    }

    @Override
    public void execute() {
        commandGroup.execute();
    }

    @Override
    public boolean isFinished() {
        return s_AlgaeMech.arePivotsAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (commandType != AlgaeCommands.DOCKED) {
            s_AlgaeMech.setInnerRollersVelocity(0);
            s_AlgaeMech.setBothFlywheelVelocity(0);
        }
    }

    public enum AlgaeCommands {
        DOCKED,
        GROUNDINTAKE,
        REEFINTAKE,
        REEFOUTTAKE,
        PROCESSOROUTTAKE,
        SHOOTNET
    }
}
