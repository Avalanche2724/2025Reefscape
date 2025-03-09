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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Controls {
  private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGLE_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond);
  private static final double STICK_DEADBAND = 0;
  private static final double SWERVEAPI_DEADBAND = 0.0;

  private final RobotContainer bot;
  private final Drivetrain drivetrain;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Climber climber;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * SWERVEAPI_DEADBAND)
          .withRotationalDeadband(MAX_ANGLE_RATE * SWERVEAPI_DEADBAND)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  // .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

  // Publisher for debug visualization of nearest branch target position
  private final StructPublisher<Pose2d> nearestBranchPose =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("NearestBranch", Pose2d.struct)
          .publish();

  // Reef level for debug visualization
  private ReefLevel debugReefLevel = ReefLevel.L1;

  public Controls(RobotContainer robot) {
    bot = robot;
    drivetrain = bot.drivetrain;
    superstructure = bot.superstructure;
    intake = bot.intake;
    climber = bot.climber;

    // Add periodic debug function that runs at 50Hz
    Robot.instance.addPeriodic(this::debugUpdateNearestBranch, 0.02);
  }

  private SwerveRequest driveBasedOnJoystick() {
    double y = -getLeftY();
    double x = getLeftX();
    // Apply radial deadband
    double hypot = Math.hypot(x, y);
    double angle = Math.atan2(y, x);
    hypot = deadband(hypot);
    x = hypot * Math.cos(angle);
    y = hypot * Math.sin(angle);

    double turnX = deadband(getRightX());

    return drive
        .withVelocityX(y * MAX_SPEED)
        .withVelocityY(-x * MAX_SPEED)
        .withRotationalRate(deadband(-turnX) * MAX_ANGLE_RATE);
  }

  public boolean isOnCoralBindings = true;

  public Command coralAlgaeCommand(Command coral, Command algae) {
    return Commands.either(coral, algae, () -> isOnCoralBindings);
  }

  public void coralAlgaeActivePresets(Trigger button, Position coral, Position algae) {
    button.whileTrue(
        coralAlgaeCommand(
            superstructure.getToPositionThenHold(coral),
            superstructure.getToPositionThenHold(algae)));
  }

  public Position nextTargetPosition = Position.OUTTAKE_L1;

  public void coralAlgaeSettingPresets(Trigger button, Position coral, Position algae) {
    button.whileTrue(
        coralAlgaeCommand(
            runOnce(() -> nextTargetPosition = coral), runOnce(() -> nextTargetPosition = algae)));
  }

  boolean currentlyAutoAligning = true;

  public BooleanSupplier createAtTargetPositionSupplier(
      DoubleSupplier meters, DoubleSupplier degrees) {
    return () -> {
      // Get current pose and expected target pose
      Pose2d currentPose = drivetrain.getState().Pose;
      Pose2d targetPose = findNearestReefBranch(ReefLevel.L1, true); // Default to left side

      // Check if we're close enough to target position and properly aligned
      double positionTolerance = meters.getAsDouble();
      double angleTolerance = degrees.getAsDouble();

      var diff = currentPose.minus(targetPose);
      boolean isAtPosition = diff.getTranslation().getNorm() < positionTolerance;
      boolean isAligned = Math.abs(diff.getRotation().getDegrees()) < angleTolerance;

      return isAtPosition && isAligned;

      // return false;
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

  public void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::driveBasedOnJoystick));
    driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    driver.back().whileTrue(superstructure.zeroElevatorCommand());

    driver.leftBumper().whileTrue(intake.runIntake());
    driver.rightBumper().whileTrue(intake.fullSend());
    /*
        driver.povDown().whileTrue(intake.runVariable(() -> -3));

        driver.povUp().whileTrue(drivetrain.wheelCharacterization());
    */

    configureDriveTuningBindings();

    configureSysidBindings();

    // AUTO ALIGN
    driver
        .leftTrigger()
        .and(this::enableAutoAlign)
        .whileTrue(driveToNearestReefBranchCommand(this::positionToReefLevel, true)); // Left side
    driver
        .rightTrigger()
        .and(this::enableAutoAlign)
        .whileTrue(driveToNearestReefBranchCommand(this::positionToReefLevel, false)); // Right side

    // Auto align bindings with automatic ejection when aligned
    var wantingToAutoAlignRn = driver.leftTrigger().and(driver.rightTrigger());
    var atTargetPositionTrigger = new Trigger(createAtTargetPositionSupplier(() -> 0.01, () -> 1));
    var nearTargetPositionTrigger = new Trigger(createAtTargetPositionSupplier(() -> 0.5, () -> 5));
    /*
        // When we are kinda near the target position while auto aligning, set superstructure position
        wantingToAutoAlignRn
            .and(nearTargetPositionTrigger)
            .whileTrue(superstructure.goToPosition(() -> nextTargetPosition));

        // When we reach the target position while auto-aligning, eject intake
        wantingToAutoAlignRn
            .and(atTargetPositionTrigger)
            .and(superstructure::atTargetPosition)
            .whileTrue(intake.fullSend().withTimeout(1));
    */
    operator.leftStick().whileTrue(superstructure.incrementWrist(() -> -1 * operator.getLeftY()));

    /*operator
            .rightStick()
            .whileTrue(superstructure.incrementElevator(() -> -0.01 * operator.getRightY()));
    */
    operator.rightStick().whileTrue(climber.runVoltage(() -> -12 * operator.getRightX()));

    operator
        .back() // left squares
        .onTrue(runOnce(() -> isOnCoralBindings = false));
    operator
        .start() // right lines
        .onTrue(runOnce(() -> isOnCoralBindings = true));

    coralAlgaeActivePresets(
        operator.leftBumper(), Position.INTAKE_CORAL_STATION, Position.INTAKE_CORAL_STATION);

    coralAlgaeActivePresets(
        operator.rightBumper(), Position.MIN_INTAKE_GROUND, Position.ALG_INTAKE_GROUND);
    coralAlgaeActivePresets(operator.a(), Position.OUTTAKE_L1, Position.ALG_PROC);
    // operator.a().whileTrue(superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));

    coralAlgaeActivePresets(operator.b(), Position.OUTTAKE_L2_LAUNCH, Position.INTAKE_ALGAE_L2);
    coralAlgaeActivePresets(operator.x(), Position.OUTTAKE_L3_LAUNCH, Position.INTAKE_ALGAE_L3);
    coralAlgaeActivePresets(operator.y(), Position.OUTTAKE_L4_LAUNCH, Position.OUTTAKE_NET);
    operator
        .leftTrigger()
        .whileTrue(superstructure.getToPositionThenHold(() -> nextTargetPosition));

    operator.rightTrigger().whileTrue(algaeLaunchSequence());
  }

  /**
   * Creates a command that drives to the nearest reef branch at the specified level and side.
   *
   * @param level The reef level to target
   * @param leftSide True for left branches, False for right branches
   * @return A command that drives to the nearest reef branch
   */
  public Command driveToNearestReefBranchCommand(Supplier<ReefLevel> level, boolean leftSide) {
    return drivetrain.driveToPosition(() -> findNearestReefBranch(level.get(), leftSide));
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
      double scoringDistance = Meters.convertFrom(38, Inches);
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

  // Debug method to periodically publish the nearest branch position to NetworkTables
  private void debugUpdateNearestBranch() {
    Pose2d targetPose = findNearestReefBranch(debugReefLevel, true);
    nearestBranchPose.set(targetPose);
  }

  public Command algaeLaunchSequence() {
    return sequence(
        superstructure.setWristPositionCommand(60),
        parallel(superstructure.elevatorAlgaeLaunch(1.5), intake.run(-4))
            .until(() -> superstructure.atLeastElevatorPosition(1.3)),
        intake.fullSend().withTimeout(0.15),
        parallel(superstructure.elevatorAlgaeLaunch(-1.5), intake.fullSend()).withTimeout(0.5),
        intake.stopIntake(),
        superstructure.goToPosition(Position.STOW));
  }

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private void configureDriveTuningBindings() {
    driver
        .pov(0)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(2).withVelocityY(0)));
    driver
        .pov(180)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-2).withVelocityY(0)));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
  }

  private void configureSuperstructureTuningBindings() {
    driver.a().whileTrue(superstructure.incrementElevator(() -> 0.005));
    driver.b().whileTrue(superstructure.incrementElevator(() -> -0.005));
    driver.x().whileTrue(superstructure.incrementWrist(() -> 0.5));
    driver.y().whileTrue(superstructure.incrementWrist(() -> -0.5));
  }

  private void configureSysidBindings() {
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.

    // final SysIdRoutine routine = superstructure.wrist.sysIdRoutine;
    var routine = drivetrain.m_sysIdRoutineToApply;
    driver.back().and(driver.y()).whileTrue(routine.dynamic(SysIdRoutine.Direction.kForward));
    driver.back().and(driver.a()).whileTrue(routine.dynamic(SysIdRoutine.Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(routine.quasistatic(SysIdRoutine.Direction.kForward));
    driver.start().and(driver.a()).whileTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  private static double deadband(double val) {
    return MathUtil.applyDeadband(val, STICK_DEADBAND);
  }

  public static final boolean keyboardMappings =
      Robot.isSimulation() && System.getProperty("os.name").toLowerCase().contains("mac");

  public double getLeftY() {
    if (keyboardMappings) {
      return driver.getLeftY() * 0.5;
    } else {
      return driver.getLeftY();
    }
  }

  public double getLeftX() {
    if (keyboardMappings) {
      return driver.getLeftX() * 0.5;
    } else {
      return driver.getLeftX();
    }
  }

  public double getRightX() {
    if (keyboardMappings) {
      return driver.getLeftTriggerAxis() * 0.5;
    } else {
      return driver.getRightX();
    }
  }
}
