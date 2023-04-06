package org.usfirst.frc4904.robot.seenoevil;

import java.util.Collections;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class UnrequiredWrapper extends WrapperCommand {
    /**
     * Wraps a command but overrides getRequirements to contain nothing.
     * 
     * @param command The command to be wrapped. This command cannot then be passed
     *                to another composition or be scheduled.
     */
    public UnrequiredWrapper(Command command) {
        super(command);
    }
    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
