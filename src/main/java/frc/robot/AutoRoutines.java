package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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
    routine.active().onTrue(superstructure.goToPosition(Position.STOW));
    /*_base_l2score
        .done()
        .onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH));
    _base_l2score
        .done()
        .onTrue(
            sequence(
                waitUntil(
                    () -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH)),
                intake.ejectIntake().withTimeout(0.5)));*/

    return routine; // TODO
  }

  public AutoRoutine StartToHumanStation() {
    final AutoRoutine routine = m_factory.newRoutine("StartLeftEdgetoHumanPlayer");
    final AutoTrajectory Start_to_reef = routine.trajectory("Start to reef");
    final AutoTrajectory reef_to_lollipop = routine.trajectory("reef to lollipop");
    final AutoTrajectory lollipop_to_reef = routine.trajectory("lollipop to reef");
    final AutoTrajectory reef_to_human_player_station =
        routine.trajectory("reef to human player station");

    routine.active().onTrue(Commands.sequence(Start_to_reef.resetOdometry(), Start_to_reef.cmd()));

    return routine;
  }
}
