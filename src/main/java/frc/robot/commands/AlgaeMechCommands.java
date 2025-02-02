package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.AlgaeMech.AlgaeStates;

public class AlgaeMechCommands extends Command {
    private final AlgaeMech s_AlgaeMech;
    private final AlgaeCommands commandType;
    private SequentialCommandGroup commandGroup;

    public AlgaeMechCommands(AlgaeMech algaeMech, AlgaeCommands commandType) {
        this.s_AlgaeMech = algaeMech;
        this.commandType = commandType;
        addRequirements(algaeMech);
    }

    @Override
    public void initialize() {
        switch (commandType) {
            case DOCKED:
                commandGroup = new SequentialCommandGroup(
                    new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.PIVOT_DOCK_SHOOT), s_AlgaeMech)
                );
                break;
            case GROUNDINTAKE, REEFINTAKE:
                AlgaeStates pivotState = (commandType == AlgaeCommands.GROUNDINTAKE) ? AlgaeStates.PIVOT_INTAKE : AlgaeStates.PIVOT_LVL2REEF;
                commandGroup = new SequentialCommandGroup(
                    new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(pivotState), s_AlgaeMech),
                    new WaitUntilCommand(s_AlgaeMech::arePivotsAtSetPoint),
                    new ParallelCommandGroup(
                        new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.FLYWHEEL_INTAKE), s_AlgaeMech),
                        new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.INNERROLLER_INTAKE), s_AlgaeMech)
                    )
                );
                break;

            case PROCESSOROUTTAKE:
                commandGroup = new SequentialCommandGroup(
                    new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.PIVOT_PROCESSOR), s_AlgaeMech),
                    new WaitUntilCommand(s_AlgaeMech::arePivotsAtSetPoint),
                    new ParallelCommandGroup(
                        new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.INNERROLLER_OUTTAKE), s_AlgaeMech),
                        new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.FLYWHEEL_SHOOT), s_AlgaeMech)
                    )
                );
                break;

            case SHOOTNET:
                commandGroup = new SequentialCommandGroup(
                    new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.PIVOT_DOCK_SHOOT), s_AlgaeMech),
                    new WaitUntilCommand(s_AlgaeMech::arePivotsAtSetPoint),
                    new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.FLYWHEEL_SHOOT), s_AlgaeMech),
                    new WaitUntilCommand(s_AlgaeMech::isFlywheelAtTargetSpeed),
                    new InstantCommand(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.INNERROLLER_OUTTAKE), s_AlgaeMech)
                );
                break;

            default:
                return;
        }
    }

    @Override
    public void execute() {
        commandGroup.execute();
    }

    @Override
    public boolean isFinished() {
        return commandGroup.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        s_AlgaeMech.stopAll();
    }

    public enum AlgaeCommands {
        DOCKED,
        GROUNDINTAKE,
        REEFINTAKE,
        PROCESSOROUTTAKE,
        SHOOTNET
    }
}
