package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.AlgaeMech.AlgaeStates;

public class AlgaeMechCommands extends Command {
    private final AlgaeMech s_AlgaeMech;
    private final AlgaeCommands commandType;
    private Command commandGroup;

    public AlgaeMechCommands(AlgaeMech algaeMech, AlgaeCommands commandType) {
        this.s_AlgaeMech = algaeMech;
        this.commandType = commandType;
        addRequirements(algaeMech);
    }

    @Override
    public void initialize() {
        switch (commandType) {
            case DOCKED:
                commandGroup = Commands.runOnce(() -> 
                s_AlgaeMech.pivotTransitionHandler(AlgaeStates.PIVOT_DOCK_SHOOT), s_AlgaeMech);
                break;
            case GROUNDINTAKE, REEFINTAKE:
                AlgaeStates pivotState = (commandType == AlgaeCommands.GROUNDINTAKE) ? AlgaeStates.PIVOT_INTAKE : AlgaeStates.PIVOT_LVL2REEF;
                commandGroup = Commands.sequence(
                    Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(pivotState), s_AlgaeMech),
                    Commands.waitUntil(s_AlgaeMech::arePivotsAtSetPoint),
                    Commands.parallel(
                        Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.FLYWHEEL_INTAKE), s_AlgaeMech),
                        Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.INNERROLLER_INTAKE), s_AlgaeMech)
                    )
                );
                break;

            case PROCESSOROUTTAKE:
                commandGroup = Commands.sequence(
                    Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.PIVOT_PROCESSOR), s_AlgaeMech),
                    Commands.waitUntil(s_AlgaeMech::arePivotsAtSetPoint),
                    Commands.parallel(
                        Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.INNERROLLER_OUTTAKE), s_AlgaeMech),
                        Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.FLYWHEEL_SHOOT), s_AlgaeMech)
                    )
                );
                break;

            case SHOOTNET:
                commandGroup = Commands.sequence(
                    Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.PIVOT_DOCK_SHOOT), s_AlgaeMech),
                    Commands.waitUntil(s_AlgaeMech::arePivotsAtSetPoint),
                    Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.FLYWHEEL_SHOOT), s_AlgaeMech),
                    Commands.waitUntil(s_AlgaeMech::isFlywheelAtTargetSpeed),
                    Commands.runOnce(() -> s_AlgaeMech.pivotTransitionHandler(AlgaeStates.INNERROLLER_OUTTAKE), s_AlgaeMech)
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
