package frc.robot;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;
import frc.robot.util.FieldConstants;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final RobotContainer container;
  private final Drivetrain drivetrain;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Climber climber;
  private final Controls controls;

  public AutoRoutines(AutoFactory factory, RobotContainer container) {
    m_factory = factory;
    this.container = container;
    drivetrain = container.drivetrain;
    superstructure = container.superstructure;
    intake = container.intake;
    climber = container.climber;
    controls = container.controls;
  }

  public Command driveToL2BranchAndScore(boolean leftSide) {
    return sequence(
        controls
            .driveToNearestReefBranchCommand(() -> FieldConstants.ReefLevel.L2, leftSide)
            .until(
                new Trigger(
                        controls.createAtTargetPositionSupplier(
                            () -> Meters.convertFrom(1, Inch), () -> 1))
                    .debounce(0.5)),
        intake.semiSend().withTimeout(0.75));
  }

  public Command intakeUntilGamePiece() {
    return drivetrain
        .brakeOnce()
        .andThen(race(intake.runIntake().withTimeout(1.9), waitUntil(intake.hasGamePieceTrigger)));
  }

  public Command waitAndL2() {
    return waitSeconds(1).andThen(superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));
  }

  public Command waitAndCoralStation() {
    return waitSeconds(1).andThen(superstructure.goToPosition(Position.INTAKE_CORAL_STATION));
  }

  public AutoRoutine verycoolpath() {
    var routine = m_factory.newRoutine("verycoolpath");

    var START_TO_BRANCH1 = routine.trajectory("CoolPath", 0);
    var BRANCH1_TO_HP = routine.trajectory("CoolPath", 1);
    var HP_TO_BRANCH2 = routine.trajectory("CoolPath", 2);
    var BRANCH2_TO_HP = routine.trajectory("CoolPath", 3);
    var HP_TO_BRANCH3 = routine.trajectory("CoolPath", 4);

    return makeCoolPath(
        routine, START_TO_BRANCH1, BRANCH1_TO_HP, HP_TO_BRANCH2, BRANCH2_TO_HP, HP_TO_BRANCH3);
  }

  public AutoRoutine coolpath2back() {
    var routine = m_factory.newRoutine("coolpath2back");

    var START_TO_BRANCH1 = routine.trajectory("CoolPathTwoBack", 0);
    var BRANCH1_TO_HP = routine.trajectory("CoolPathTwoBack", 1);
    var HP_TO_BRANCH2 = routine.trajectory("CoolPathTwoBack", 2);
    var BRANCH2_TO_HP = routine.trajectory("CoolPathTwoBack", 3);
    var HP_TO_BRANCH3 = routine.trajectory("CoolPathTwoBack", 4);

    return makeCoolPath(
        routine, START_TO_BRANCH1, BRANCH1_TO_HP, HP_TO_BRANCH2, BRANCH2_TO_HP, HP_TO_BRANCH3);
  }

  public AutoRoutine coolpath2backsamecage() {
    var routine = m_factory.newRoutine("coolpath2backsamecage");

    var START_TO_BRANCH1 = routine.trajectory("CoolPathTwoBack_SameCage", 0);
    var BRANCH1_TO_HP = routine.trajectory("CoolPathTwoBack_SameCage", 1);
    var HP_TO_BRANCH2 = routine.trajectory("CoolPathTwoBack_SameCage", 2);
    var BRANCH2_TO_HP = routine.trajectory("CoolPathTwoBack_SameCage", 3);
    var HP_TO_BRANCH3 = routine.trajectory("CoolPathTwoBack_SameCage", 4);

    return makeCoolPath(
        routine, START_TO_BRANCH1, BRANCH1_TO_HP, HP_TO_BRANCH2, BRANCH2_TO_HP, HP_TO_BRANCH3);
  }

  private AutoRoutine makeCoolPath(
      AutoRoutine routine,
      AutoTrajectory START_TO_BRANCH1,
      AutoTrajectory BRANCH1_TO_HP,
      AutoTrajectory HP_TO_BRANCH2,
      AutoTrajectory BRANCH2_TO_HP,
      AutoTrajectory HP_TO_BRANCH3) {
    routine
        .active()
        .onTrue(
            (Robot.isSimulation() ? START_TO_BRANCH1.resetOdometry() : print("Starting auto"))
                .andThen(START_TO_BRANCH1.cmd()));
    routine.active().onTrue(superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));

    START_TO_BRANCH1
        .atTimeBeforeEnd(0.4)
        .onTrue(sequence(driveToL2BranchAndScore(false), BRANCH1_TO_HP.spawnCmd()));

    BRANCH1_TO_HP.active().onTrue(waitAndCoralStation());
    BRANCH1_TO_HP.done().onTrue(sequence(intakeUntilGamePiece(), HP_TO_BRANCH2.spawnCmd()));

    HP_TO_BRANCH2.active().onTrue(waitAndL2());
    HP_TO_BRANCH2
        .atTimeBeforeEnd(0.4)
        .onTrue(sequence(driveToL2BranchAndScore(true), BRANCH2_TO_HP.spawnCmd()));

    BRANCH2_TO_HP.active().onTrue(waitAndCoralStation());
    BRANCH2_TO_HP.done().onTrue(sequence(intakeUntilGamePiece(), HP_TO_BRANCH3.spawnCmd()));

    HP_TO_BRANCH3.active().onTrue(waitAndL2());
    HP_TO_BRANCH3
        .atTimeBeforeEnd(0.4)
        .onTrue(
            sequence(
                driveToL2BranchAndScore(false),
                Commands.print("Complete!"),
                superstructure.goToPositionOnce(Position.STOW)));

    return routine; // TODO
  }

  public AutoRoutine l1forauto_LEFT() {
    var routine = m_factory.newRoutine("l1forauto_LEFT");
    var l1forauto = routine.trajectory("NEW_FORGO_L1_LEFT");

    routine
        .active()
        .onTrue(
            sequence(
                l1forauto.resetOdometry(),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                waitSeconds(2.5),
                l1forauto.cmd(),
                waitSeconds(1.0),
                intake.ejectIntake().withTimeout(1),
                superstructure.goToPosition(Position.STOW)));

    return routine; // TODO
  }

  public AutoRoutine l1forauto_RIGHT() {
    var routine = m_factory.newRoutine("l1forauto_RIGHT");
    var l1forauto = routine.trajectory("NEW_FORGO_L1_RIGHT");

    routine
        .active()
        .onTrue(
            sequence(
                l1forauto.resetOdometry(),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                waitSeconds(2.5),
                l1forauto.cmd(),
                waitSeconds(1.0),
                intake.ejectIntake().withTimeout(1),
                superstructure.goToPosition(Position.STOW)));

    return routine; // TODO
  }

  public AutoRoutine l1forauto_ORIG() {
    var routine = m_factory.newRoutine("l1forauto_ORIG");
    var l1forauto = routine.trajectory("NEW_FORWARDGO");
    /*
    routine.active().onTrue(l1forauto.resetOdometry().andThen(l1forauto.cmd()));
    routine.active().onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L1));
    // 1forauto.done().onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L1));
    l1forauto
        .done()
        .onTrue(
            sequence(
                waitUntil(() -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L1)),
                waitSeconds(3.0),
                intake.ejectIntake().withTimeout(1)));*/
    routine
        .active()
        .onTrue(
            sequence(
                l1forauto.resetOdometry(),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                waitSeconds(2.5),
                l1forauto.cmd(),
                waitSeconds(1.0),
                intake.ejectIntake().withTimeout(1),
                superstructure.goToPosition(Position.STOW)));

    return routine; // TODO
  }

  public AutoRoutine l1forauto_PUSH() {
    var routine = m_factory.newRoutine("l1forauto_PUSH");
    var l1forauto = routine.trajectory("NEW_FORGO_L1_PUSH");

    routine
        .active()
        .onTrue(
            sequence(
                l1forauto.resetOdometry(),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                waitSeconds(2.5),
                l1forauto.cmd(),
                waitSeconds(1.0),
                intake.ejectIntake().withTimeout(1),
                superstructure.goToPosition(Position.STOW)));

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
    var reef_to_HPSS = routine.trajectory("reef to HPSS");
    var HPSS_to_reef = routine.trajectory("HPSS to reef");
    // var reef_to_human_player_station = routine.trajectory("reef to human player station");

    if (Start_to_reef == null || HPSS_to_reef == null || reef_to_HPSS == null) {
      throw new RuntimeException("One or more trajectories are missing!");
    }

    /*  routine.active().onTrue(Start_to_reef.resetOdometry().andThen(Start_to_reef.cmd()));
    routine.active().onTrue(superstructure.goToPosition(Position.STOW));
    Start_to_reef.done()
        .onTrue(superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L2_LAUNCH));
    Start_to_reef.done()
        .onTrue(
            sequence(
                waitUntil(
                    () -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L2_LAUNCH)),
                intake.ejectIntake().withTimeout(1),
                reef_to_lollipop.spawnCmd())); */
    routine.active().onTrue(Commands.sequence(Start_to_reef.resetOdometry(), Start_to_reef.cmd()));
    Start_to_reef.done()
        .onTrue(
            sequence(
                Commands.print("Reached Reef"),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L2_LAUNCH),
                intake.ejectIntake().withTimeout(0.5),
                Commands.print("Ejecting Intake"),
                reef_to_HPSS.spawnCmd()));

    /*routine.active().onTrue(reef_to_lollipop.cmd());
    routine.active().onTrue(superstructure.goToPosition(Position.STOW));
    reef_to_lollipop
        .done()
        .onTrue(superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1));
    reef_to_lollipop
        .done()
        .onTrue(
            sequence(
                Commands.print("Reached Lollipop"),
                waitUntil(() -> superstructure.atPosition(Superstructure.Position.OUTTAKE_L1)),
                intake.runIntake().withTimeout(0.5),
                Commands.print("Running Intake"),
                lollipop_to_reef.spawnCmd())); */

    reef_to_HPSS
        .done()
        .onTrue(
            sequence(
                Commands.print("Reached Lollipop"),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                intake.runIntake().withTimeout(0.5),
                Commands.print("Running Intake"),
                HPSS_to_reef.spawnCmd()));

    /*routine.active().onTrue(lollipop_to_reef.cmd());
    lollipop_to_reef
        .done()
        .onTrue(superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1));
    lollipop_to_reef
        .done()
        .onTrue(
            sequence(
                Commands.print("Returning to Reef"),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                intake.ejectIntake().withTimeout(0.5),
                Commands.print("Ejecting Again"))); */
    HPSS_to_reef.done()
        .onTrue(
            sequence(
                Commands.print("Returning to Reef"),
                superstructure.goToPositionOnce(Superstructure.Position.OUTTAKE_L1),
                intake.ejectIntake().withTimeout(0.5),
                Commands.print("Ejecting Again")));
    /*reef_to_human_player_station
    .done()
    .onTrue(
        sequence(
            Commands.print("Heading to Human Player Station"),
            superstructure.goToPositionOnce(Superstructure.Position.STOW),
            Commands.print("Auto Routine Complete"))); */

    return routine;
  }
}
