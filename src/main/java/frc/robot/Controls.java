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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;

import java.util.Map;
import java.util.function.Supplier;

public class Controls {
  private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGLE_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond);
  private static final double STICK_DEADBAND = 0.0;
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

  public void coralAlgaePresets(Trigger button, Position coral, Position algae) {
    button.whileTrue(
        coralAlgaeCommand(
            superstructure.getToPositionThenHold(coral),
            superstructure.getToPositionThenHold(algae)));
  }

  public void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::driveBasedOnJoystick));
    driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    driver.back().whileTrue(superstructure.zeroElevatorCommand());

    driver.leftBumper().whileTrue(intake.runIntake());
    driver.rightBumper().whileTrue(intake.fullSend());
    driver.rightTrigger().whileTrue(intake.run(-1.5));

    // Add bind for auto-drive to nearest reef branch
    driver.leftTrigger().whileTrue(driveToNearestReefBranchCommand(ReefLevel.L1));

    configureSysidBindings();

    operator.leftStick().whileTrue(superstructure.incrementWrist(() -> -1 * operator.getLeftY()));
    operator
        .rightStick()
        .whileTrue(superstructure.incrementElevator(() -> -0.01 * operator.getRightY()));

    operator
        .back() // left squares
        .onTrue(runOnce(() -> isOnCoralBindings = false));
    operator
        .start() // right lines
        .onTrue(runOnce(() -> isOnCoralBindings = true));

    coralAlgaePresets(
        operator.rightBumper(), Position.MIN_INTAKE_GROUND, Position.ALG_INTAKE_GROUND);
    coralAlgaePresets(operator.a(), Position.OUTTAKE_L1, Position.ALG_PROC);
    // operator.a().whileTrue(superstructure.goToPosition(Position.OUTTAKE_L2_LAUNCH));

    coralAlgaePresets(operator.b(), Position.OUTTAKE_L2_LAUNCH, Position.INTAKE_ALGAE_L2);
    coralAlgaePresets(operator.x(), Position.OUTTAKE_L3_LAUNCH, Position.INTAKE_ALGAE_L3);
    coralAlgaePresets(operator.y(), Position.OUTTAKE_L4_LAUNCH, Position.OUTTAKE_NET);

    operator.rightTrigger().whileTrue(algaeLaunchSequence());
  }

  /**
   * Creates a command that drives to the nearest reef branch at the specified level.
   * 
   * @param level The reef level to target
   * @return A command that drives to the nearest reef branch
   */
  public Command driveToNearestReefBranchCommand(ReefLevel level) {
    return drivetrain.driveToPosition(findNearestReefBranch(level));
  }

  /**
   * Creates a supplier that provides the Pose2d of the nearest reef branch at the specified level.
   * The returned pose will be positioned 20 inches away from the branch and facing the branch.
   * 
   * @param level The reef level to target
   * @return A supplier that provides the proper robot pose for scoring at the nearest reef branch
   */
  private Supplier<Pose2d> findNearestReefBranch(ReefLevel level) {
    return () -> {
      Pose2d currentPose = drivetrain.getState().Pose;
      Pose2d nearestBranch = null;
      double minDistance = Double.MAX_VALUE;
      
      // Iterate through all branches and find the nearest one at the specified level
      for (Map<ReefLevel, Pose2d> branchMap : FieldConstants.Reef.branchPositions2d) {
        Pose2d branchPose = branchMap.get(level);
        if (branchPose != null) {
          // Calculate distance from current position to this branch
          double distance = Math.hypot(
              currentPose.getX() - branchPose.getX(),
              currentPose.getY() - branchPose.getY());
          
          // Check if this branch is closer than the current closest
          if (distance < minDistance) {
            minDistance = distance;
            nearestBranch = branchPose;
          }
        }
      }
      
      // If we found a branch, calculate the proper scoring position
      if (nearestBranch != null) {
        // The branch poses face outward, so we need to face the opposite direction to face the branch
        Rotation2d branchRotation = nearestBranch.getRotation();
        
        // Calculate a position that is 20 inches (0.508 meters) away from the branch
        // in the direction opposite to the branch's orientation
        double scoringDistance = 0.508; // 20 inches in meters
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
      return currentPose;
    };
  }

  // Debug method to periodically publish the nearest branch position to NetworkTables
  private void debugUpdateNearestBranch() {
    Pose2d targetPose = findNearestReefBranch(debugReefLevel).get();
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
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private void configureDriveTuningBindings() {
    driver
        .pov(0)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driver
        .pov(180)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

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

    final SysIdRoutine routine = superstructure.wrist.sysIdRoutine;
    driver.back().and(driver.y()).whileTrue(routine.dynamic(SysIdRoutine.Direction.kForward));
    driver.back().and(driver.x()).whileTrue(routine.dynamic(SysIdRoutine.Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(routine.quasistatic(SysIdRoutine.Direction.kForward));
    driver.start().and(driver.x()).whileTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse));
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
