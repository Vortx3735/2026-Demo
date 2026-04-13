package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import java.util.function.Supplier;

public class CommandFactory {
  public static Command runTunnelAndHopperCommand(Tunnel tunnel, Hopper hopper) {
    return Commands.parallel(tunnel.intakeCommand(), hopper.intakeCommand());
  }

  public static Command manualShootCommand(Flywheel flywheel, Hopper hopper, Tunnel tunnel) {
    return Commands.parallel(
            flywheel.shootCommand(),
            Commands.sequence(
                new WaitUntilCommand(flywheel.isAtSpeed()),
                Commands.parallel(hopper.intakeCommand(), tunnel.intakeCommand())))
        .withName("manual shoot command group");
  }

  public static Command manualShootCommandAtSpeed(
      Flywheel flywheel, Hopper hopper, Tunnel tunnel, Supplier<Double> speed) {
    return Commands.parallel(
            flywheel.shootCommand(speed.get()),
            Commands.sequence(
                new WaitUntilCommand(flywheel.isAtSpeed()),
                Commands.parallel(hopper.intakeCommand(), tunnel.intakeCommand())))
        .withName("manual shoot command group");
  }

  public static Command shootCommand(
      Flywheel flywheel, Tunnel tunnel, Hopper hopper, Intake intake, Supplier<Double> targetRPS) {
    return Commands.parallel(
            flywheel.shootCommand(targetRPS),
            Commands.sequence(
                Commands.deadline(
                    new WaitUntilCommand(flywheel.isAtSpeed()), hopper.intakeSlowCommand()),
                Commands.parallel(hopper.intakeCommand(), tunnel.intakeCommand())),
            new RunCommand(() -> intake.setSpeed(1), intake)
            // ,Commands.sequence(new WaitUntilCommand(flywheel.isAtSpeed()),
            // tunnel.intakeCommand())
            )
        .withName("shoot command group");
  }

  public static Command shootCommandBackwardsHopper(
      Flywheel flywheel, Tunnel tunnel, Hopper hopper, Intake intake, Supplier<Double> targetRPS) {
    return Commands.parallel(
            flywheel.shootCommand(targetRPS),
            new RunCommand(() -> intake.setSpeed(1), intake),
            Commands.sequence(
                new WaitUntilCommand(flywheel.isAtSpeed()),
                Commands.parallel(tunnel.intakeCommand(), hopper.outtakeCommand())))
        .withName("shoot command group");
  }

  public static Command intakeCommand(Intake intake, Hopper hopper) {
    return Commands.parallel(intake.intakeCommand(), hopper.intakeCommand())
        .withName("intake command group");
  }

  public static Command outtakeCommand(Intake intake, Hopper hopper) {
    return Commands.parallel(intake.outtakeCommand(), hopper.outtakeCommand())
        .withName("outtake command group");
  }

  public static Command clearJamsCommand(Tunnel tunnel, Hopper hopper) {
    return Commands.parallel(tunnel.outtakeCommand(), hopper.outtakeCommand())
        .withName("clear jams");
  }

  public static Command runTunnelWithoutHopper(Tunnel tunnel, Hopper hopper) {
    return Commands.parallel(tunnel.intakeCommand(), hopper.outtakeCommand())
        .withName("clear jams");
  }
}
