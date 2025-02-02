package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.CoralMech.CoralStates;

public class CoralMechCommands extends Command {
    private final CoralMech s_CoralMech;
    private final CoralCommands commandType;
    private CoralStates initialState;
    private CoralStates finalState;
    private Command commandGroup;

    public CoralMechCommands(CoralMech coralMech, CoralCommands commandType) {
        this.s_CoralMech = coralMech;
        this.commandType = commandType;
        addRequirements(coralMech);
    }
    @Override
    public void initialize() {
        // Set initial and final states based on the command type
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
        commandGroup = Commands.sequence(
            Commands.runOnce(() -> s_CoralMech.coralTransitionHandler(initialState), s_CoralMech),
            Commands.waitUntil(s_CoralMech::isWristAtSetpoint),
            Commands.runOnce(() -> {
                if (finalState != null) {
                    s_CoralMech.coralTransitionHandler(finalState);
                }
            }, s_CoralMech)
        );
    }
    
    @Override
    //use commands. to fix this
    public void execute() {
        commandGroup.execute();
    }
    
    @Override
    public boolean isFinished() {
        return commandGroup.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (commandType != CoralCommands.DOCKED) {
            s_CoralMech.setBothRollersVoltage(0);
            s_CoralMech.stopAllMotors();
        }
    }
    

    public enum CoralCommands {
        DOCKED,
        HPINTAKE,
        REEFOUTTAKE,
        LVL1OUTTAKE,
    }
}
