package org.usfirst.frc4904.robot.seenoevil;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * A command composition that runs a set of commands in parallel, ending only when a specific
 * command (the "deadline") ends, interrupting all other commands that are still running at that
 * point.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("removal")
@Deprecated // broken because it both registers composed AND does not require
public class BasedDeadline extends CommandGroupBase {
  // maps commands in this composition to whether they are still running
  private final Map<Command, Boolean> m_commands = new HashMap<>();
  private boolean m_runWhenDisabled = true;
  private boolean m_finished = true;
  private Command m_deadline;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelSelf;

  /**
   * Humble ParallelDeadlineGroup: inturrupts self and requires nothing
   * 
   *  Creates a new ParallelDeadlineGroup. The given commands (including the deadline) will be
   * executed simultaneously. The composition will finish when the deadline finishes, interrupting
   * all other still-running commands. If the composition is interrupted, only the commands still
   * running will be interrupted.
   *
   * @param deadline the command that determines when the composition ends
   * @param commands the commands to be executed
   */
  public BasedDeadline(String name, Command deadline, Command... commands) {
    m_deadline = deadline;
    setName(name);
    addCommands(commands);
    if (!m_commands.containsKey(deadline)) {
      addCommands(deadline);
    }
  }
  public BasedDeadline(Command deadline, Command... commands) { this("Deadline: " + deadline.getName(), deadline, commands); }

  /**
   * Sets the deadline to the given command. The deadline is added to the group if it is not already
   * contained.
   *
   * @param deadline the command that determines when the group ends
   */
  public void setDeadline(Command deadline) {
    if (!m_commands.containsKey(deadline)) {
      addCommands(deadline);
    }
    m_deadline = deadline;
  }

  @Override
  public final void addCommands(Command... commands) {
    if (!m_finished) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      if (!Collections.disjoint(command.getRequirements(), m_requirements)) {
        throw new IllegalArgumentException(
            "Multiple commands in a parallel group cannot" + "require the same subsystems");
      }
      m_commands.put(command, false);
      m_runWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
      commandRunning.getKey().initialize();
      commandRunning.setValue(true);
    }
    m_finished = false;
  }

  @Override
  public final void execute() {
    for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
      if (!commandRunning.getValue()) {
        continue;
      }
      commandRunning.getKey().execute();
      if (commandRunning.getKey().isFinished()) {
        commandRunning.getKey().end(false);
        commandRunning.setValue(false);
        if (commandRunning.getKey().equals(m_deadline)) {
          m_finished = true;
        }
      }
    }
  }

  @Override
  public final void end(boolean interrupted) {
    for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
      if (commandRunning.getValue()) {
        commandRunning.getKey().end(true);
      }
    }
  }

  @Override
  public final boolean isFinished() {
    return m_finished;
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty("deadline", m_deadline::getName, null);
  }
}