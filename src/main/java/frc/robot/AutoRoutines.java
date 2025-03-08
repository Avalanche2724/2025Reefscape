package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.superstructure.Superstructure.Position;

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
    _base_l2score.done().onTrue(superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));
    _base_l2score
        .done()
        .onTrue(
            sequence(
                waitUntil(() -> superstructure.atPosition(Position.OUTTAKE_L2_LAUNCH)),
                intake.ejectIntake().withTimeout(0.5)));

    return null; // TODO
  }
}
