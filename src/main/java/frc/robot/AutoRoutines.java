package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final RobotContainer container;

  public AutoRoutines(AutoFactory factory, RobotContainer container) {
    m_factory = factory;
    this.container = container;
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
    // _base_l2score.done().onTrue(container.superstructure.goToPosition());

    return null; // TODO
  }
}
