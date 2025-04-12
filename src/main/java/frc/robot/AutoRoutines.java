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
    return driveToBranchAndScore(leftSide, FieldConstants.ReefLevel.L2);
  }

  public Command driveToBranchAndScore(boolean leftSide, FieldConstants.ReefLevel level) {
    return race(
        controls.driveToNearestReefBranchCommand(() -> level, leftSide),
        sequence(
            waitUntil(
                new Trigger(
                        controls.createAtTargetPositionSupplier(
                            () -> Meters.convertFrom(1, Inch), () -> 1))
                    .and(superstructure::atTargetPosition)
                    .debounce(0.6)),
            intake.semiSend().withTimeout(0.5)));
  }

  public Command intakeUntilGamePiece() {
    return intakeUntilGamepieceForever().withTimeout(2.0);
  }

  public Command intakeUntilGamepieceForever() {
    return /*drivetrain
           .brakeOnce()
           .andThen(*/ (race(
        intake.leftMajority(), waitUntil(intake.hasGamePieceTrigger)));
  }

  public Command intakeForever() {
    return intake.leftMajority();
  }

  public Command waitAndL2() {
    return waitSeconds(1).andThen(superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));
  }

  public Command waitAndStow() {
    return waitSeconds(0.4).andThen(superstructure.goToPosition(Position.STOW));
  }

  public Command waitAndSemiStow() {
    return waitSeconds(0.4).andThen(superstructure.goToPosition(Position.SEMISTOW));
  }

  public Command waitAndScoryStow() {
    return waitSeconds(0.4).andThen(superstructure.goToPosition(Position.SCORYSTOW));
  }

  public Command waitAndCoralStation() {
    return waitSeconds(1).andThen(superstructure.goToPosition(Position.INTAKE_CORAL_STATION));
  }

  public Command noWaitAndL2() {
    return (superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));
  }

  public Command noWaitAndL3() {
    return (superstructure.goToPosition(Position.OUTTAKE_L3_LAUNCH));
  }

  public Command noWaitAndStowWhileOuttakeThenSemiStow() {
    return (superstructure.goToPosition(Position.STOW))
        .alongWith(intake.semiSend())
        .withTimeout(0.4)
        .andThen(superstructure.goToPosition(Position.SEMISTOW));
  }

  public Command noWaitAndCoralStation() {
    return (superstructure.goToPosition(Position.INTAKE_CORAL_STATION));
  }

  public AutoRoutine verycoolpath() {
    var routine = m_factory.newRoutine("verycoolpath");

    var START_TO_BRANCH1 = routine.trajectory("CoolPath", 0);
    var BRANCH1_TO_HP = routine.trajectory("CoolPath", 1);
    var HP_TO_BRANCH2 = routine.trajectory("CoolPath", 2);
    var BRANCH2_TO_HP = routine.trajectory("CoolPath", 3);
    var HP_TO_BRANCH3 = routine.trajectory("CoolPath", 4);

    return makeCoolPath2(
        routine, START_TO_BRANCH1, BRANCH1_TO_HP, HP_TO_BRANCH2, BRANCH2_TO_HP, HP_TO_BRANCH3);
  }

  public AutoRoutine coolpath2back() {
    var routine = m_factory.newRoutine("coolpath2back");

    var START_TO_BRANCH1 = routine.trajectory("CoolPathTwoBack", 0);
    var BRANCH1_TO_HP = routine.trajectory("CoolPathTwoBack", 1);
    var HP_TO_BRANCH2 = routine.trajectory("CoolPathTwoBack", 2);
    var BRANCH2_TO_HP = routine.trajectory("CoolPathTwoBack", 3);
    var HP_TO_BRANCH3 = routine.trajectory("CoolPathTwoBack", 4);

    return makeCoolPath2(
        routine, START_TO_BRANCH1, BRANCH1_TO_HP, HP_TO_BRANCH2, BRANCH2_TO_HP, HP_TO_BRANCH3);
  }

  public AutoRoutine coolpath2backsamecage() {
    var routine = m_factory.newRoutine("coolpath2backsamecage");

    var START_TO_BRANCH1 = routine.trajectory("CoolPathTwoBack_SameCage", 0);
    var BRANCH1_TO_HP = routine.trajectory("CoolPathTwoBack_SameCage", 1);
    var HP_TO_BRANCH2 = routine.trajectory("CoolPathTwoBack_SameCage", 2);
    var BRANCH2_TO_HP = routine.trajectory("CoolPathTwoBack_SameCage", 3);
    var HP_TO_BRANCH3 = routine.trajectory("CoolPathTwoBack_SameCage", 4);

    return makeCoolPath2(
        routine, START_TO_BRANCH1, BRANCH1_TO_HP, HP_TO_BRANCH2, BRANCH2_TO_HP, HP_TO_BRANCH3);
  }

  private AutoRoutine makeCoolPath2(
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
    routine.active().onTrue(superstructure.goToPosition(Position.SCORYSTOW));

    START_TO_BRANCH1.atTimeBeforeEnd(1.1).onTrue(noWaitAndL2());
    START_TO_BRANCH1
        .atTimeBeforeEnd(0.4)
        .onTrue(sequence(driveToL2BranchAndScore(false), BRANCH1_TO_HP.spawnCmd()));

    BRANCH1_TO_HP.active().onTrue(noWaitAndStowWhileOuttakeThenSemiStow());
    BRANCH1_TO_HP.atTimeBeforeEnd(1.1).onTrue(noWaitAndCoralStation());
    BRANCH1_TO_HP
        .atTimeBeforeEnd(0.5)
        .onTrue(sequence(intakeUntilGamePiece(), HP_TO_BRANCH2.spawnCmd()));

    BRANCH1_TO_HP.done().onTrue(drivetrain.brakeOnce());

    HP_TO_BRANCH2.active().onTrue(waitAndScoryStow());
    HP_TO_BRANCH2.atTimeBeforeEnd(1.1).onTrue(noWaitAndL2());
    HP_TO_BRANCH2
        .atTimeBeforeEnd(0.4)
        .onTrue(sequence(driveToL2BranchAndScore(true), BRANCH2_TO_HP.spawnCmd()));

    BRANCH2_TO_HP.active().onTrue(noWaitAndStowWhileOuttakeThenSemiStow());
    BRANCH2_TO_HP.atTimeBeforeEnd(1.1).onTrue(noWaitAndCoralStation());
    BRANCH2_TO_HP
        .atTimeBeforeEnd(0.5)
        .onTrue(sequence(intakeUntilGamePiece(), HP_TO_BRANCH3.spawnCmd()));

    BRANCH2_TO_HP.done().onTrue(drivetrain.brakeOnce());

    HP_TO_BRANCH3.active().onTrue(waitAndScoryStow());
    HP_TO_BRANCH3.atTimeBeforeEnd(1.1).onTrue(noWaitAndL2());
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
                drivetrain.brakeOnce(),
                waitSeconds(1.0),
                intake.ejectL1intake().withTimeout(1),
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

  public AutoRoutine BestMiddlePathLOL() {
    var routine = m_factory.newRoutine("BestMiddlePathFR");

    var START_TO_BRANCH1 = routine.trajectory("Extremely_Cool_Middle_Path", 0);
    var BRANCH1_TO_TAKEALG = routine.trajectory("Extremely_Cool_Middle_Path", 1);
    var ALGAE_TO_NET = routine.trajectory("Extremely_Cool_Middle_Path", 2);
    var NET_TO_SideAlgae = routine.trajectory("Extremely_Cool_Middle_Path", 3);
    var SIDEALGAE_TO_NET = routine.trajectory("Extremely_Cool_Middle_Path", 4);

    routine
        .active()
        .onTrue(
            (Robot.isSimulation() ? START_TO_BRANCH1.resetOdometry() : print("Starting auto"))
                .andThen(waitSeconds(0.2))
                .andThen(START_TO_BRANCH1.cmd()));
    routine.active().onTrue(superstructure.goToPosition(Position.OUTTAKE_L3_LAUNCH));

    START_TO_BRANCH1
        .atTimeBeforeEnd(0.4)
        .onTrue(
            sequence(
                driveToBranchAndScore(true, FieldConstants.ReefLevel.L3),
                superstructure.goToPositionOnce(Position.INTAKE_ALGAE_L2),
                Commands.waitSeconds(0.3),
                BRANCH1_TO_TAKEALG.spawnCmd()));

    BRANCH1_TO_TAKEALG.active().onTrue(intakeForever());
    BRANCH1_TO_TAKEALG.done().onTrue(ALGAE_TO_NET.cmd());
    ALGAE_TO_NET.atTime(0.75).onTrue(superstructure.goToPositionOnce(Position.SEMISEMISTOW));

    ALGAE_TO_NET
        .done()
        .onTrue(sequence(Commands.print("Reached Lollipop"), controls.driveToAlgaeLaunchCmd()));
    ALGAE_TO_NET
        .done()
        .onTrue(
            waitUntil(
                    controls.createAtTargetPositionSupplier(
                        () -> Meters.convertFrom(15, Inch), () -> 8))
                .andThen(controls.algaeLaunchSequence())
                .andThen(NET_TO_SideAlgae.spawnCmd()));

    NET_TO_SideAlgae
        .done()
        .onTrue(sequence(superstructure.goToPositionOnce(Position.INTAKE_ALGAE_L3))
            .andThen(intake.semiSend().withTimeout(0.5)));

    return routine;
  }
}
