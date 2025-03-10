package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final RobotContainer container;
  private final Drivetrain drivetrain;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Climber climber;

  public AutoRoutines(AutoFactory factory, RobotContainer container) {
    m_factory = factory;
    this.container = container;
    drivetrain = container.drivetrain;
    superstructure = container.superstructure;
    intake = container.intake;
    climber = container.climber;
  }

  public AutoRoutine simplePathAuto() {
    var routine = m_factory.newRoutine("test auto");
    var simplePath = routine.trajectory("New Path");

    routine
        .active()
        .onTrue(
            Commands.print("RUNNING AUTO ROUTINE")
                .andThen(simplePath.resetOdometry().andThen(simplePath.cmd())));

    return routine;
  }

  public AutoRoutine simplePathAuto2() {
    var routine = m_factory.newRoutine("test auto");
    var _base_l2score = routine.trajectory("_base_l2score");

    routine.active().onTrue(_base_l2score.cmd());
    _base_l2score
        .done()
        .onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH));
    _base_l2score
        .done()
        .onTrue(
            sequence(
                waitUntil(
                    () -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH)),
                intake.ejectIntake().withTimeout(0.5)));

    return null; // TODO reer
  }

  public AutoRoutine StartToHumanStation() {
    var routine = m_factory.newRoutine("test auto");
    if (routine == null) {
      throw new RuntimeException("Routine creation failed!");
    }

    var Start_to_reef = routine.trajectory("Start to reef");
    var reef_to_lollipop = routine.trajectory("reef to lollipop");
    var lollipop_to_reef = routine.trajectory("lollipop to reef");
    var reef_to_human_player_station = routine.trajectory("reef to human player station");

    if (Start_to_reef == null
        || reef_to_lollipop == null
        || lollipop_to_reef == null
        || reef_to_human_player_station == null) {
      throw new RuntimeException("One or more trajectories are missing!");
    }

    routine.active().onTrue(Start_to_reef.cmd());
    Start_to_reef.done()
        .onTrue(
            sequence(
                Commands.print("Reached Reef"),
                superstructure.goToPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH),
                waitUntil(
                    () -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH)),
                intake.ejectIntake().withTimeout(0.5),
                Commands.print("Ejecting Intake"),
                reef_to_lollipop.spawnCmd()));

    routine.active().onTrue(reef_to_lollipop.cmd());
    reef_to_lollipop
        .done()
        .onTrue(
            sequence(
                Commands.print("Reached Lollipop"),
                superstructure.goToPosition(Superstructure.Position.OUTTAKE_L1),
                waitUntil(() -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L1)),
                intake.runIntake().withTimeout(0.5),
                Commands.print("Running Intake"),
                lollipop_to_reef.spawnCmd()));

    routine.active().onTrue(lollipop_to_reef.cmd());
    lollipop_to_reef
        .done()
        .onTrue(
            sequence(
                Commands.print("Returning to Reef"),
                superstructure.goToPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH),
                waitUntil(
                    () -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH)),
                intake.ejectIntake().withTimeout(0.5),
                Commands.print("Ejecting Again"),
                reef_to_human_player_station.spawnCmd()));

    routine.active().onTrue(reef_to_human_player_station.cmd());
    reef_to_human_player_station
        .done()
        .onTrue(
            sequence(
                Commands.print("Heading to Human Player Station"),
                superstructure.goToPosition(Superstructure.Position.STOW),
                waitUntil(() -> superstructure.atPosition(Superstructure.Position.STOW)),
                Commands.print("Auto Routine Complete")));

    return routine;
  }
}
