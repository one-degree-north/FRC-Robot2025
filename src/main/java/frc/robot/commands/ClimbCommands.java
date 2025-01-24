package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbStates;

public class ClimbCommands extends Command {
    private final Climb s_Climb;
    private final ClimbStates commandType;
    private Command commandToRun;

    public ClimbCommands(Climb climb, ClimbStates commandType) {
        this.s_Climb = climb;
        this.commandType = commandType;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        commandToRun = Commands.run(() -> s_Climb.climbTransitionHandler(commandType), s_Climb);
    }

    @Override
    public void execute() {
        commandToRun.execute();
    }

    @Override
    public boolean isFinished() {
        return commandToRun.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        s_Climb.stopClimb();
    }
}
