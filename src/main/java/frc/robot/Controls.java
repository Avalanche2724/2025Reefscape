package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;

public class Controls {
  private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGLE_RATE = RotationsPerSecond.of(1).in(RadiansPerSecond);
  private static final double STICK_DEADBAND = 0.001;
  private static final double SWERVEAPI_DEADBAND = 0.0;

  private final RobotContainer bot;
  private final Drivetrain drivetrain;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Climber climber;

  private final CommandXboxController driver = new CommandXboxController(0);

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

  public void configureBindings() {
    configureSuperstructureTuningBindings();
    /*
    driver.a().onTrue(superstructure.goToPosition(Superstructure.Position.OUTTAKE_L2));
    driver.b().onTrue(superstructure.goToPosition(Superstructure.Position.INTAKE_VERTICAL_CORAL));
    driver.x().onTrue(superstructure.stop());
    */
    /*driver.a().whileTrue(intake.run(3));
    driver.b().whileTrue(intake.run(-3));
    driver.x().whileTrue(intake.run(12));
    driver.y().whileTrue(intake.run(-12));*/
    /*driver.a().whileTrue(intake.runVariable(() -> driver.getRightTriggerAxis() * 12));
    driver.b().whileTrue(intake.runVariable(() -> driver.getRightTriggerAxis() * -12));
    driver.x().whileTrue(intake.run(2).withTimeout(0.2).andThen(intake.fullSend()));
    driver.y().whileTrue(intake.spinny());*/
    // configureSuperstructureTuningBindings();

    // driver.a().

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> {
              getLeftY();
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
            }));
    driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // configureSysidBindings();
    /*driver.a().whileTrue(intake.runIntake());
    driver.b().whileTrue(intake.ejectIntake());
    driver.x().whileTrue(intake.semiSpinny());
    driver.y().whileTrue(intake.spinny());*/
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
    // driver.a().whileTrue(superstructure.incrementElevator(0.005));
    // driver.b().whileTrue(superstructure.incrementElevator(-0.005));
    driver.x().whileTrue(superstructure.incrementWrist(0.5));
    driver.y().whileTrue(superstructure.incrementWrist(-0.5));
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
}
