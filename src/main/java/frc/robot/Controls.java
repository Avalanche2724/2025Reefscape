package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Position;

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

  public Controls(RobotContainer robot) {
    bot = robot;
    drivetrain = bot.drivetrain;
    superstructure = bot.superstructure;
    intake = bot.intake;
    climber = bot.climber;
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

    configureSysidBindings();

    operator.leftStick().whileTrue(superstructure.incrementWrist(() -> -1 * operator.getLeftY()));
    operator
        .rightStick()
        .whileTrue(superstructure.incrementElevator(() -> -0.01 * operator.getRightY()));

    coralAlgaePresets(
        operator.rightBumper(), Position.MIN_INTAKE_GROUND, Position.ALG_INTAKE_GROUND);
    coralAlgaePresets(operator.a(), Position.OUTTAKE_L1, Position.ALG_PROC);

    coralAlgaePresets(operator.b(), Position.OUTTAKE_L2_LAUNCH, Position.INTAKE_ALGAE_L2);
    coralAlgaePresets(operator.x(), Position.OUTTAKE_L3_LAUNCH, Position.INTAKE_ALGAE_L3);
    coralAlgaePresets(operator.y(), Position.OUTTAKE_L4_LAUNCH, Position.OUTTAKE_NET);

    operator.rightTrigger().whileTrue(algaeLaunchSequence());
  }

  public Command algaeLaunchSequence() {
    return sequence(
        parallel(
                superstructure.elevatorAlgaeLaunch(1.5),
                intake.run(-4),
                superstructure.setWristPositionCommand(60))
            .until(() -> superstructure.atElevatorPosition(1.3)),
        intake.fullSend().withTimeout(0.5),
        parallel(superstructure.elevatorAlgaeLaunch(-1.5), intake.fullSend()).withTimeout(1.0),
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
