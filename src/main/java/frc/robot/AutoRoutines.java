package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;

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

  public AutoRoutine l1forauto() {
    var routine = m_factory.newRoutine("l1forauto");
    var l1forauto = routine.trajectory("NEW_FORGO_L1");

    routine.active().onTrue(l1forauto.resetOdometry().andThen(l1forauto.cmd()));
    routine.active().onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L1));
    // 1forauto.done().onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L1));
    l1forauto
        .done()
        .onTrue(
            sequence(
                waitUntil(() -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L1)),
                waitSeconds(3.0),
                intake.ejectIntake().withTimeout(1)));

    return routine; // TODO
  }

  public AutoRoutine simplePathAuto() {
    var routine = m_factory.newRoutine("forward going auto");
    var simplePath = routine.trajectory("NEW_goforward");

    routine
        .active()
        .onTrue(
            Commands.print("RUNNING AUTO ROUTINE")
                .andThen(simplePath.resetOdometry().andThen(simplePath.cmd())));

    return routine;
  }

  public AutoRoutine yesPathAuto() {
    var routine = m_factory.newRoutine("yes auto");
    var simplePath = routine.trajectory("!Start to reef");
    var nextPath = routine.trajectory("!reef to hp");

    routine
        .active()
        .onTrue(
            Commands.print("RUNNING AUTO ROUTINE")
                .andThen(simplePath.resetOdometry().andThen(simplePath.cmd())));

    simplePath.done().onTrue(nextPath.cmd());

    return routine;
  }

  public AutoRoutine simplePathAuto2() {
    var routine = m_factory.newRoutine("simple path auto");
    var _base_l2score = routine.trajectory("Start to F branch");

    routine.active().onTrue(_base_l2score.resetOdometry().andThen(_base_l2score.cmd()));
    routine.active().onTrue(superstructure.goToPosition(Position.STOW));
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

    return routine; // TODO
  }

  public AutoRoutine brrforward() {
    var routine = m_factory.newRoutine("brr forward");
    var brrforward = routine.trajectory("NEW_FORWARDGO");

    routine.active().onTrue(brrforward.resetOdometry().andThen(brrforward.cmd()));
    routine.active().onTrue(superstructure.goToPosition(Position.STOW));
    brrforward
        .done()
        .onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH));
    brrforward
        .done()
        .onTrue(
            sequence(
                waitUntil(
                    () -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH)),
                waitSeconds(3.0),
                intake.ejectIntake().withTimeout(1)));

    return routine; // TODO
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

    routine.active().onTrue(Commands.sequence(Start_to_reef.resetOdometry(), Start_to_reef.cmd()));
    Start_to_reef.done()
        .onTrue(
            sequence(
                Start_to_reef.resetOdometry(),
                Commands.print("Reached Reef"),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L2_LAUNCH),
                intake.ejectIntake().withTimeout(0.5),
                Commands.print("Ejecting Intake"),
                reef_to_lollipop.spawnCmd()));

    reef_to_lollipop
        .done()
        .onTrue(
            sequence(
                Commands.print("Reached Lollipop"),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                intake.runIntake().withTimeout(0.5),
                Commands.print("Running Intake"),
                lollipop_to_reef.spawnCmd()));

    lollipop_to_reef
        .done()
        .onTrue(
            sequence(
                Commands.print("Returning to Reef"),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L2_LAUNCH),
                intake.ejectIntake().withTimeout(0.5),
                Commands.print("Ejecting Again"),
                reef_to_human_player_station.spawnCmd()));

    reef_to_human_player_station
        .done()
        .onTrue(
            sequence(
                Commands.print("Heading to Human Player Station"),
                superstructure.goToPositionOnce(Superstructure.Position.STOW),
                Commands.print("Auto Routine Complete")));

    return routine;
  }
}
