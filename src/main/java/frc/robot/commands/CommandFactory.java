package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class CommandFactory {
  public static Command exampleCommand() {
    return Commands.parallel(
        RobotContainer.flywheel.setVelocityPIDCommand(),
        Commands.sequence(
            new WaitUntilCommand(() -> RobotContainer.flywheel.isAtSpeed()),
            RobotContainer.tunnel.runTunnelCommand(false)));
  }
}
