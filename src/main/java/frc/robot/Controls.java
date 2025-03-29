package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.util.GenericGamepad;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Controls {
  // Constants and stuff
  private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGLE_RATE = RotationsPerSecond.of(2.0).in(RadiansPerSecond);
  private static final double STICK_DEADBAND = 0;
  private static final double SWERVEAPI_DEADBAND = 0.0;
  // Subsystems and things
  private final RobotContainer bot;
  private final Drivetrain drivetrain;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Climber climber;
  // Controllers
  public final GenericGamepad driver = GenericGamepad.from(0);
  private final GenericGamepad operator = GenericGamepad.from(1);
  // Swerve requests
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * SWERVEAPI_DEADBAND)
          .withRotationalDeadband(MAX_ANGLE_RATE * SWERVEAPI_DEADBAND)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  // Publisher for debug visualization of nearest branch target position and other auto align stuff
  private final StructPublisher<Pose2d> nearestBranchPose =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("NearestBranch", Pose2d.struct)
          .publish();
  public boolean currentlyAutoAligning = false;
  // Internal state for controls
  public boolean isOnCoralBindings = true;
  public Position nextTargetPosition = Position.OUTTAKE_L1;
  public Pose2d lastPoseForAutoAlign = null;

  public Controls(RobotContainer robot) {
    bot = robot;
    drivetrain = bot.drivetrain;
    superstructure = bot.superstructure;
    intake = bot.intake;
    climber = bot.climber;

    Robot.instance.addPeriodic(this::periodic, 0.02);
  }

  public Command tuneDrivetrainStaticFriction() {
    double[] speed = new double[] {0};
    return runOnce(() -> speed[0] = 0)
        .andThen(drivetrain.applyRequest(() -> drive.withVelocityX(speed[0] += 0.0005)));
  }

  //
  public void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::driveBasedOnJoystick));
    driver.rightMiddle.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    driver.leftMiddle.whileTrue(superstructure.zeroElevatorCommand());

    driver.leftBumper.whileTrue(intake.leftMajority());
    driver.rightBumper.whileTrue(intake.semiSend());

    configureSysidBindings();

    driver.y.whileTrue(intake.fullSend());
    driver.b.whileTrue(intake.run(2));
    driver.a.whileTrue(intake.runIntake());

    coralAlgaeActivePresets(driver.b, Position.MIN_INTAKE_GROUND, Position.ALG_INTAKE_GROUND);

    // driver.povLeft.whileTrue(tuneDrivetrainStaticFriction());
    // configureDriveTuningBindings();
    driver.povUp.whileTrue(drivetrain.wheelCharacterization());

    // configureSysidBindings();

    // AUTO ALIGN

    driver
        .leftTriggerB
        .and(this::enableAutoAlign)
        .whileTrue(driveToNearestReefBranchCommand(this::positionToReefLevel, true)); // Left side
    driver
        .rightTriggerB
        .and(this::enableAutoAlign)
        .whileTrue(driveToNearestReefBranchCommand(this::positionToReefLevel, false)); // Right side

    // Auto align bindings with automatic ejection when aligned
    var wantingToAutoAlignRn = driver.leftTriggerB.or(driver.rightTriggerB);
    var atTargetPositionTrigger =
        new Trigger(createAtTargetPositionSupplier(() -> Meters.convertFrom(1.1, Inch), () -> 1.1))
            .debounce(0.45);

    wantingToAutoAlignRn
        .and(atTargetPositionTrigger)
        .and(superstructure::atTargetPosition)
        .onTrue(
            intake
                .semiSend()
                .withTimeout(0.6)
                .alongWith(
                    startEnd(() -> driver.rumble.accept(0.2), () -> driver.rumble.accept(0))
                        .withTimeout(0.2)));

    // var atTargetPositionTrigger = new Trigger(createAtTargetPositionSupplier(() -> 0.05, () ->
    // 2));
    // var nearTargetPositionTrigger = new Trigger(createAtTargetPositionSupplier(() -> 0.5, () ->
    // 5));
    /*
        // When we are kinda near the target position while auto aligning, set superstructure position
        wantingToAutoAlignRn
            .and(nearTargetPositionTrigger)
            .whileTrue(
                Commands.print("AT TARG POS SET POS")
                    .andThen(superstructure.goToPosition(() -> nextTargetPosition)));

        // When we reach the target position while auto-aligning, eject intake
        wantingToAutoAlignRn
            .and(atTargetPositionTrigger)
            .and(() -> superstructure.atPosition(nextTargetPosition))
            .whileTrue(
                intake
                    .fullSend()
                    .withTimeout(1)
                    .andThen(superstructure.goToPositionOnce(Position.STOW)));
    */
    // operator.leftJoystickPushed.whileTrue(
    //    superstructure.incrementWrist(() -> -1 * operator.getLeftY()));

    operator
        .rightJoystickPushed
        .negate()
        .and(() -> Math.abs(operator.getRightY()) > 0.7)
        .whileTrue(
            superstructure.incrementElevator(
                () -> -0.01 * MathUtil.applyDeadband(operator.getRightY(), 0.7)));

    operator.rightJoystickPushed.whileTrue(
        climber.runVoltage(() -> -6 * MathUtil.applyDeadband(operator.getRightX(), 0.25)));

    operator.leftMiddle // left squares
        .onTrue(runOnce(() -> isOnCoralBindings = false));
    operator.rightMiddle // right lines
        .onTrue(runOnce(() -> isOnCoralBindings = true));

    coralAlgaeActivePresets(
        operator.leftBumper, Position.INTAKE_CORAL_STATION, Position.INTAKE_CORAL_STATION);

    coralAlgaeActivePresets(
        operator.rightBumper, Position.MIN_INTAKE_GROUND, Position.ALG_INTAKE_GROUND);
    coralAlgaeActivePresets(operator.a, Position.OUTTAKE_L1, Position.ALG_PROC);
    // operator.a().whileTrue(superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));

    coralAlgaeActivePresets(operator.b, Position.OUTTAKE_L2_LAUNCH, Position.INTAKE_ALGAE_L2);
    coralAlgaeActivePresets(operator.x, Position.OUTTAKE_L3_LAUNCH, Position.INTAKE_ALGAE_L3);
    coralAlgaeActivePresets(operator.y, Position.OUTTAKE_L4_LAUNCH, Position.OUTTAKE_NET);
    // operator.leftTriggerB.whileTrue(superstructure.getToPositionThenHold(() ->
    // nextTargetPosition));
    operator.leftTriggerB.whileTrue(superstructure.goToPosition(Position.STOW));

    operator.rightTriggerB.whileTrue(algaeLaunchSequence());
  }

  private SwerveRequest driveBasedOnJoystick() {
    double y = -driver.getLeftY();
    double x = driver.getLeftX();
    // Apply radial deadband
    double hypot = Math.hypot(x, y);
    double angle = Math.atan2(y, x);
    hypot = deadband(hypot);
    x = hypot * Math.cos(angle);
    y = hypot * Math.sin(angle);

    double turnX = deadband(driver.getRightX());

    return drive
        .withVelocityX(y * MAX_SPEED)
        .withVelocityY(-x * MAX_SPEED)
        .withRotationalRate(deadband(-turnX) * MAX_ANGLE_RATE);
  }

  private void configureDriveTuningBindings() {
    /*
        driver.povDown().whileTrue(intake.runVariable(() -> -3));

        driver.povUp().whileTrue(drivetrain.wheelCharacterization());
    */
    driver.povUp.whileTrue(
        drivetrain.applyRequest(() -> forwardStraight.withVelocityX(2).withVelocityY(0)));
    driver.povDown.whileTrue(
        drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-2).withVelocityY(0)));

    driver.a.whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b.whileTrue(
        drivetrain.applyRequest(
            () ->
                point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
  }

  private void configureSysidBindings() {
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.

    // final SysIdRoutine routine = superstructure.wrist.sysIdRoutine;
    var routine = superstructure.elevator.sysIdRoutine;
    driver.leftMiddle.and(driver.y).whileTrue(routine.dynamic(SysIdRoutine.Direction.kForward));
    driver.leftMiddle.and(driver.a).whileTrue(routine.dynamic(SysIdRoutine.Direction.kReverse));
    driver
        .rightMiddle
        .and(driver.y)
        .whileTrue(routine.quasistatic(SysIdRoutine.Direction.kForward));
    driver
        .rightMiddle
        .and(driver.a)
        .whileTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public void coralAlgaeActivePresets(Trigger button, Position coral, Position algae) {
    button.whileTrue(
        coralAlgaeCommand(
            superstructure.getToPositionThenHold(coral),
            superstructure.getToPositionThenHold(algae)));
  }

  public void coralAlgaeSettingPresets(Trigger button, Position coral, Position algae) {
    button.whileTrue(
        coralAlgaeCommand(
            runOnce(() -> nextTargetPosition = coral), runOnce(() -> nextTargetPosition = algae)));
  }

  // Commands

  public Command coralAlgaeCommand(Command coral, Command algae) {
    return Commands.either(coral, algae, () -> isOnCoralBindings);
  }

  public Command algaeLaunchSequence() {
    return sequence(
        intake.holdIntake().raceWith(superstructure.elevatorAlgaeLaunchSetup()),
        Commands.print("Algae launch sequence done 1"),
        intake.fullSend().withTimeout(0.5),
        Commands.print("Algae launch sequence done 2"),
        superstructure.elevatorAlgaeLaunchPostscript().alongWith(intake.fullSend()));
  }

  public BooleanSupplier createAtTargetPositionSupplier(
      DoubleSupplier meters, DoubleSupplier degrees) {
    return () -> {
      // Get current pose and expected target pose
      Pose2d currentPose = drivetrain.getState().Pose;
      Pose2d targetPose = lastPoseForAutoAlign;
      if (targetPose == null) {
        return false;
      }

      // Check if we're close enough to target position and properly aligned
      double positionTolerance = meters.getAsDouble();
      double angleTolerance = degrees.getAsDouble();

      var diff = currentPose.minus(targetPose);
      boolean isAtPosition = diff.getTranslation().getNorm() < positionTolerance;
      boolean isAligned =
          Math.toDegrees(Math.abs(MathUtil.angleModulus(diff.getRotation().getRadians())))
              < angleTolerance;

      return isAtPosition && isAligned;
    };
  }

  public ReefLevel positionToReefLevel() {
    return switch (nextTargetPosition) {
      case OUTTAKE_L1 -> ReefLevel.L1;
      case OUTTAKE_L2_LAUNCH, INTAKE_ALGAE_L2 -> ReefLevel.L2;
      case OUTTAKE_L3_LAUNCH, INTAKE_ALGAE_L3 -> ReefLevel.L3;
      case OUTTAKE_L4_LAUNCH, OUTTAKE_NET -> ReefLevel.L4;
      default -> null;
    };
  }

  public boolean enableAutoAlign() {
    return positionToReefLevel() != null;
  }

  /**
   * Creates a command that drives to the nearest reef branch at the specified level and side.
   *
   * @param level The reef level to target
   * @param leftSide True for left branches, False for right branches
   * @return A command that drives to the nearest reef branch
   */
  public Command driveToNearestReefBranchCommand(Supplier<ReefLevel> level, boolean leftSide) {
    return drivetrain.driveToPosition(
        () -> lastPoseForAutoAlign = findNearestReefBranch(level.get(), leftSide));
  }

  /**
   * Finds the nearest reef branch at the specified level and side. Returns Pose2d to target
   *
   * @param level The reef level to target
   * @param leftSide True for left branches, False for right branches
   * @return The proper robot pose for scoring at the nearest reef branch
   */
  private Pose2d findNearestReefBranch(ReefLevel level, boolean leftSide) {
    Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d nearestBranch = null;
    double minAngleDifference = Double.MAX_VALUE;
    List<Map<ReefLevel, Pose2d>> branchPositions2d =
        AllianceFlipUtil.shouldFlip()
            ? FieldConstants.Reef.redBranchPositions2d
            : FieldConstants.Reef.branchPositions2d;
    // Based on FieldConstants.java, branches alternate right/left in the list:
    // Even indexes (0, 2, 4...) are right branches
    // Odd indexes (1, 3, 5...) are left branches
    leftSide = AllianceFlipUtil.shouldFlip() ^ leftSide;
    for (int i = 0; i < branchPositions2d.size(); i++) {
      // Skip if this branch is not on the requested side
      boolean isBranchOnLeftSide = (i % 2 == 1); // Odd indexes are left branches
      if (isBranchOnLeftSide != leftSide) {
        continue;
      }
      Map<ReefLevel, Pose2d> branchMap = branchPositions2d.get(i);
      Pose2d branchPose = branchMap.get(level);
      if (branchPose != null) {
        // Calculate the angle robot needs to have when facing the branch (180° from branch angle)
        Rotation2d branchRotation = branchPose.getRotation();
        Rotation2d targetRobotRotation = branchRotation.plus(Rotation2d.fromDegrees(180));

        // Calculate the angle difference between robot's current rotation and target rotation
        double angleDifference =
            Math.abs(currentPose.getRotation().minus(targetRobotRotation).getDegrees());

        // Check if this branch requires less turning than the current best
        if (angleDifference < minAngleDifference) {
          minAngleDifference = angleDifference;
          nearestBranch = branchPose;
        }
      }
    }
    // If we found a branch, calculate the proper scoring position
    if (nearestBranch != null) {
      // The branch poses face outward, so we need to face the opposite direction to face the branch
      Rotation2d branchRotation = nearestBranch.getRotation();
      // Calculate a position that is away from the branch
      // in the direction opposite to the branch's orientation
      double scoringDistance = Meters.convertFrom(33, Inches);
      if (level == ReefLevel.L4) {
        scoringDistance = Meters.convertFrom(25, Inches);
      }

      double offsetX = scoringDistance * Math.cos(branchRotation.getRadians());
      double offsetY = scoringDistance * Math.sin(branchRotation.getRadians());
      // Create the robot scoring position: offset from branch and facing toward the branch
      return new Pose2d(
          nearestBranch.getX() + offsetX,
          nearestBranch.getY() + offsetY,
          branchRotation.plus(Rotation2d.fromDegrees(180)) // Face toward the branch
          );
    }
    // If we didn't find a branch, return the current pose
    System.out.println("Warning: could not find nearest reef branch");
    return currentPose;
  }

  // Periodic stuff
  private void periodic() {
    Pose2d targetPose = findNearestReefBranch(ReefLevel.L1, true);
    nearestBranchPose.set(targetPose);
  }

  private double deadband(double val) {
    return MathUtil.applyDeadband(val, STICK_DEADBAND);
  }
}
