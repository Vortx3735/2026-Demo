package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tunnel;
import frc.robot.subsystems.shooter.Flywheel;

public class CommandFactory {
  public static Command shootCommand(Flywheel flywheel, Tunnel tunnel) {
    return Commands.parallel(
            flywheel.setVelocityPIDCommand(),
            Commands.sequence(
                new WaitUntilCommand(() -> flywheel.isAtSpeed()), tunnel.runTunnelCommand(false)))
        .withName("shoot command group");
  }

  public static Command intakeCommand(Intake intake, Hopper hopper) {
    return Commands.parallel(intake.intakeCommand(), hopper.runHopperCommand(false))
        .withName("intake command group");
  }

  private static Command outtakeSetSpeed(Intake intake, Hopper hopper) {
    return Commands.parallel(
    Commands.run(() -> intake.setSpeed(-(intake.getSpeed()))),
    Commands.run(() -> hopper.setHopperSpeed(-(hopper.getHopperSpeed())))
  );
  }

  public static Command outtakeCommand(Intake intake, Hopper hopper) {
    // Reverse intake and hopper speeds
    intake.setSpeed(-(intake.getSpeed()));
    hopper.setHopperSpeed(-(hopper.getHopperSpeed()));

    return Commands.sequence(
      Commands.parallel(intake.intakeCommand(), hopper.runHopperCommand(false))
        .withName("intake command group"),
      outtakeSetSpeed(intake, hopper));
  }

  public static Command clearJamsCommand(Tunnel tunnel, Hopper hopper) {
    return Commands.parallel(tunnel.runTunnelCommand(true), hopper.runHopperCommand(true))
        .withName("clear jams");
  }
}
