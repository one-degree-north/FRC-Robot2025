package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    
        // Execute initial state transition
        coralMech.coralTransitionHandler(initialState);
    }
    
    @Override
    public void execute() {
        if (finalState != null && coralMech.isWristAtSetpoint()) {
            coralMech.coralTransitionHandler(finalState);
        }
    }
    
    @Override
    public boolean isFinished() {
        return coralMech.isWristAtSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (commandType != CoralCommands.DOCKED) {
            coralMech.setBothRollersVoltage(0);
            coralMech.stopAllMotors();
        }
    }
    

    public enum CoralCommands {
        DOCKED,
        HPINTAKE,
        REEFOUTTAKE,
        LVL1OUTTAKE,
    }
}
