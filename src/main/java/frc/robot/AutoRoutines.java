package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.ShooterCommands;

public class AutoRoutines {
  private final double shotTime = 6.0;
  private final AutoFactory m_factory;
  private final RobotContainer m_container;

  public AutoRoutines(AutoFactory factory, RobotContainer container) {
    m_factory = factory;
    m_container = container;

    m_factory
        .bind("intake", CommandFactory.intakeCommand(m_container.intake, m_container.hopper))
        .bind(
            "stop intake",
            Commands.parallel(m_container.intake.stopCommand(), m_container.hopper.stopCommand()));
  }

  // returns the command group for aiming in auton
  private Command aim() {
    return ShooterCommands.AimEverythingToHub(
            m_container.turret, m_container.hood, () -> m_container.drive.getTurretPose(), 65)
        .withTimeout(0.5);
  }

  // returns the command group for shooting in auton
  private Command shoot() {
    return Commands.sequence(
        aim(),
        ShooterCommands.ShootFromDistance(
            m_container.led,
            m_container.flywheel,
            m_container.hood,
            m_container.tunnel,
            m_container.hopper,
            m_container.intake,
            () -> m_container.drive.getTurretPose(),
            65));
    // CommandFactory.clearJamsCommand(m_container.tunnel, m_container.hopper).withTimeout(0.75));
  }

  //   public AutoRoutine exampleRoutine() {
  //     // Creates a routine called "example" and loads a trajectory. The trajectory is essentially
  // the
  //     // path the robot will take during auton. Look at
  //     // https://choreo.autos/usage/editing-paths/#generating for an example.
  //     final AutoRoutine routine = m_factory.newRoutine("example");
  //     final AutoTrajectory exampleTraj = routine.trajectory("ExampleTraj");

  //     // If a routine was a method, you could think of this as its body.
  //     // The "routine.active())" trigger is essentially the "entrance" to a routine.
  //     // Here, you can sequence commands for the routine.
  //     routine
  //         .active()
  //         .onTrue(
  //             // Since onTrue only has one parameter, you need to use Commands.sequence to
  // schedule
  //             // more than one command.
  //             Commands.sequence(
  //                 // No semicolons! Since you are passing arguments, it is only one statement. A
  //                 // semicolon only comes at the end of a statement. Also, since you are passing
  //                 // multiple arguments, you need commas.
  //                 exampleTraj.resetOdometry(),
  //                 exampleTraj
  //                     .cmd(), // Schedule the trajectory (make the robot move on the trajectory)
  //                 // Run commands in parallel (at the same time)
  //                 m_container.drive.stopCommand(),
  //                 CommandFactory.intakeCommand(m_container.intake, m_container.hopper),
  //                 // Run flywheel then stop
  //                 ShooterCommands.ShootFromDistance(
  //                     m_container.flywheel,
  //                     m_container.tunnel,
  //                     m_container.hopper,
  //                     m_container.intake,
  //                     () -> m_container.drive.getTurretPose(),
  //                     65)));
  //     // Stop intake and indexer
  //     // m_container.intake.stopCommand(),
  //     // m_container.hopper.stopCommand();
  //     // If a routine was a method, you could think of this as its body.
  //     // The "routine.active())" trigger is essentially the "entrance" to a routine.
  //     // Here, you can sequence commands for the routine.
  //     routine
  //         .active()
  //         .onTrue(
  //             // Since onTrue only has one parameter, you need to use Commands.sequence to
  // schedule
  //             // more than one command.
  //             Commands.sequence(
  //                 // No semicolons! Since you are passing arguments, it is only one statement. A
  //                 // semicolon only comes at the end of a statement. Also, since you are passing
  //                 // multiple arguments, you need commas.
  //                 exampleTraj.resetOdometry(),
  //                 exampleTraj
  //                     .cmd(), // Schedule the trajectory (make the robot move on the trajectory)
  //                 // Run commands in parallel (at the same time)
  //                 m_container.drive.stopCommand(),
  //                 CommandFactory.intakeCommand(m_container.intake, m_container.hopper),
  //                 // Run flywheel then stop
  //                 ShooterCommands.ShootFromDistance(
  //                     m_container.flywheel,
  //                     m_container.tunnel,
  //                     m_container.hopper,
  //                     m_container.intake,
  //                     () -> m_container.drive.getTurretPose(),
  //                     65)));
  //     // Stop intake and indexer
  //     // m_container.intake.stopCommand(),
  //     // m_container.hopper.stopCommand();

  //     return routine;
  //   }
  //     return routine;
  //   }

  public AutoRoutine rightDoubleCenter() {
    final AutoRoutine routine = m_factory.newRoutine("rightDoubleCenter");
    final AutoTrajectory center = routine.trajectory("RightCenter");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    routine.active().onTrue(Commands.sequence(center.resetOdometry(), center.cmd()));

    center.doneFor(shotTime).whileTrue(shoot());
    center.doneDelayed(shotTime).onTrue(reset.cmd());

    reset.done().onTrue(center.cmd());

    // center bindings will call again

    return routine;
  }

  public AutoRoutine leftDoubleCenter() {
    final AutoRoutine routine = m_factory.newRoutine("leftDoubleCenter");
    final AutoTrajectory center = routine.trajectory("LeftCenter");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    routine.active().onTrue(Commands.sequence(center.resetOdometry(), center.cmd()));

    center.doneFor(shotTime).whileTrue(shoot());
    center.doneDelayed(shotTime).onTrue(reset.cmd());

    reset.done().onTrue(center.cmd());

    // center bindings will call again

    return routine;
  }

  public AutoRoutine behindHub() {
    final AutoRoutine routine = m_factory.newRoutine("behindHub");
    final AutoTrajectory behindHubTraj = routine.trajectory("BehindHub");

    routine.active().onTrue(Commands.sequence(behindHubTraj.resetOdometry(), behindHubTraj.cmd()));

    behindHubTraj.doneFor(shotTime).whileTrue(shoot());

    return routine;
  }

  //   public AutoRoutine whydidntidothisearlier() {
  //     final AutoRoutine routine = m_factory.newRoutine("ok");
  //     final AutoTrajectory behindHubTraj = routine.trajectory("test");

  //     routine.active().onTrue(Commands.sequence(behindHubTraj.resetOdometry(),
  // behindHubTraj.cmd()));

  //   }

  // Contests half of neutral zone then shoots, then does it again
  public AutoRoutine leftDblShortCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftDblShortCenterContest");
    final AutoTrajectory initialDriveBack = routine.trajectory("LeftInitialDriveBack");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("LeftShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    routine
        .active()
        .onTrue(Commands.sequence(initialDriveBack.resetOdometry(), initialDriveBack.cmd()));

    // short
    initialDriveBack.doneFor(3).whileTrue(shoot());
    initialDriveBack.doneDelayed(3).onTrue(driveToMiddle.cmd());
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddle.cmd());

    // short bindings will call again

    return routine;
  }

  public AutoRoutine test() {
    final AutoRoutine routine = m_factory.newRoutine("test");
    final AutoTrajectory test = routine.trajectory("test");

    routine.active().onTrue(Commands.sequence(test.resetOdometry(), test.cmd()));

    return routine;
  }

  // Contests half of neutral zone then shoots, then does it again
  public AutoRoutine rightDblShortCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightDblShortCenterContest");
    final AutoTrajectory initialDriveBack = routine.trajectory("RightInitialDriveBack");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("RightShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    routine
        .active()
        .onTrue(Commands.sequence(initialDriveBack.resetOdometry(), initialDriveBack.cmd()));

    initialDriveBack.doneFor(3).whileTrue(shoot());
    initialDriveBack.doneDelayed(3).onTrue(driveToMiddle.cmd());
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddle.cmd());

    // short bindings will call again

    return routine;
  }

  public AutoRoutine rightLucas() {
    final AutoRoutine routine = m_factory.newRoutine("rightLucas");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightDriveVeryCenter");
    final AutoTrajectory driveBack = routine.trajectory("RightDriveBackLucas");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    final AutoTrajectory driveToHub = routine.trajectory("RightDriveToHub");
    final AutoTrajectory driveBehindHub = routine.trajectory("RightDriveBehindHub");
    // final AutoTrajectory driveBackHub = routine.trajectory("RightDriveBackHub");

    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToHub.cmd());
    driveToHub.done().onTrue(driveBehindHub.cmd());

    // short bindings will call again
    driveBehindHub
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    // driveBehindHub.done().onTrue(driveBack.cmd());

    // driveBack.doneFor(shotTime).whileTrue(shoot());
    return routine;
  }

  public AutoRoutine rightLucasSingle() {
    final AutoRoutine routine = m_factory.newRoutine("rightLucas");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightDriveVeryCenterSingle");
    final AutoTrajectory driveBack = routine.trajectory("RightDriveBackSingle");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    final AutoTrajectory driveToHub = routine.trajectory("RightDriveToHub");
    final AutoTrajectory driveBehindHub = routine.trajectory("RightDriveBehindHubSingle");
    final AutoTrajectory driveBackHub = routine.trajectory("RightDriveBackHubSingle");

    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToHub.cmd());
    driveToHub.doneFor(shotTime).onTrue(driveBehindHub.cmd());

    // short bindings will call again
    driveBehindHub
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveBehindHub.done().onTrue(driveBackHub.cmd());

    driveBackHub.doneFor(shotTime).whileTrue(shoot());
    return routine;
  }

  public AutoRoutine rightLucasOneCycle() {
    final AutoRoutine routine = m_factory.newRoutine("rightLucasOneCycle");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightDriveVeryCenter");
    final AutoTrajectory driveBack = routine.trajectory("RightDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    final AutoTrajectory driveToCenter = routine.trajectory("RightDriveToCenter");

    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToCenter.cmd());

    return routine;
  }

  public AutoRoutine rightBehindHubOnly() {
    final AutoRoutine routine = m_factory.newRoutine("rightBehindHub");
    final AutoTrajectory driveToHub = routine.trajectory("RightDriveOverBump");
    final AutoTrajectory driveBehindHub = routine.trajectory("RightDriveBehindHub");
    final AutoTrajectory driveBack = routine.trajectory("RightDriveBackLucas");

    routine
      .active()
      .onTrue(
        Commands.sequence(
          new WaitCommand(hubOnlyDelay), driveToHub.resetOdometry(), driveToHub.cmd())
        );

    driveBehindHub
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveBehindHub.done().onTrue(driveBack.cmd());

    driveBack.done().whileTrue(shoot());

    return routine;
  }

  public AutoRoutine leftLucas() {
    final AutoRoutine routine = m_factory.newRoutine("leftLucas");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftDriveVeryCenter");
    final AutoTrajectory driveBack = routine.trajectory("LeftDriveBackLucas");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    final AutoTrajectory driveToHub = routine.trajectory("LeftDriveToHub");
    final AutoTrajectory driveBehindHub = routine.trajectory("LeftDriveBehindHub");
    // final AutoTrajectory driveBackHub = routine.trajectory("LeftDriveBackHub");

    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToHub.cmd());
    driveToHub.doneFor(shotTime).onTrue(driveBehindHub.cmd());

    // short bindings will call again
    driveBehindHub
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    // driveBehindHub.done().onTrue(driveBackHub.cmd());

    // driveBackHub.done().whileTrue(shoot());
    return routine;
  }

  public AutoRoutine leftLucasSingle() {
    final AutoRoutine routine = m_factory.newRoutine("leftLucas");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftDriveVeryCenterSingle");
    final AutoTrajectory driveBack = routine.trajectory("LeftDriveBackSingle");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    final AutoTrajectory driveToHub = routine.trajectory("LeftDriveToHub");
    final AutoTrajectory driveBehindHub = routine.trajectory("LeftDriveBehindHubSingle");
    final AutoTrajectory driveBackHub = routine.trajectory("LeftDriveBackHubSingle");

    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToHub.cmd());
    driveToHub.doneFor(shotTime).onTrue(driveBehindHub.cmd());

    // short bindings will call again
    driveBehindHub
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveBehindHub.done().onTrue(driveBackHub.cmd());

    driveBackHub.doneFor(shotTime).whileTrue(shoot());
    return routine;
  }

  public AutoRoutine leftLucasOneCycle() {
    final AutoRoutine routine = m_factory.newRoutine("leftLucasOneCycle");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftDriveVeryCenter");
    final AutoTrajectory driveBack = routine.trajectory("LeftDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    final AutoTrajectory driveToCenter = routine.trajectory("LeftDriveToCenter");

    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(shotTime).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(shotTime).onTrue(reset.cmd());
    reset.done().onTrue(driveToCenter.cmd());

    return routine;
  }

  public AutoRoutine hpSimple() {
    final AutoRoutine routine = m_factory.newRoutine("hpRight");
    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");

    routine.active().onTrue(Commands.sequence(moveToHP.resetOdometry(), moveToHP.cmd()));

    // move to shooting position 2s after it gets there. gives human player some time
    moveToHP.done().onTrue(shoot());

    return routine;
  }

  public AutoRoutine standstill() {
    final AutoRoutine routine = m_factory.newRoutine("standstill");
    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");
    routine
        .active()
        .whileTrue(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(15),
                    Commands.sequence(
                        Commands.parallel(
                            ShooterCommands.AimEverythingToHub(
                                m_container.turret,
                                m_container.hood,
                                () -> m_container.drive.getPose(),
                                65),
                            Commands.sequence(
                                new WaitCommand(1),
                                ShooterCommands.ShootFromDistance(
                                    m_container.led,
                                    m_container.flywheel,
                                    m_container.hood,
                                    m_container.tunnel,
                                    m_container.hopper,
                                    m_container.intake,
                                    () -> m_container.drive.getTurretPose(),
                                    65))))),
                moveToHP.resetOdometry(),
                moveToHP.cmd()));

    return routine;
  }

  public AutoRoutine standstillunjam() {
    final AutoRoutine routine = m_factory.newRoutine("standstill");

    routine.active().whileTrue(shoot());

    return routine;
  }

  public AutoRoutine moveTwoMeters() {
    AutoRoutine routine = m_factory.newRoutine("moveTwoMeters");

    AutoTrajectory traj = routine.trajectory("straight1meter");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }
}
