package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class Controls {
  private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGLE_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private static final double STICK_DEADBAND = 0.01;
  private static final double SWERVEAPI_DEADBAND = 0.0001;

  private final RobotContainer bot;
  private final Drivetrain drivetrain;
  private final Elevator elevator;
  private final Intake intake;
  private final Wrist wrist;
  private final Climber climber;
  private final LED led;

  private final CommandXboxController driver = new CommandXboxController(0);

  // Swerve requests necessary for drivetrain control
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * SWERVEAPI_DEADBAND)
          .withRotationalDeadband(MAX_ANGLE_RATE * SWERVEAPI_DEADBAND)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  public Controls(RobotContainer robot) {
    bot = robot;
    drivetrain = bot.drivetrain;
    elevator = bot.elevator;
    intake = bot.intake;
    wrist = bot.wrist;
    led = bot.led;
    climber = bot.climber;
  }

  public void configureBindings() {

    driver.a().whileTrue(wrist.incrementMotorPositionForTesting(0.001).repeatedly());
    driver.b().whileTrue(wrist.incrementMotorPositionForTesting(-0.001).repeatedly());
    driver.x().whileTrue(elevator.incrementMotorPositionForTesting(0.04).repeatedly());
    driver.y().whileTrue(elevator.incrementMotorPositionForTesting(-0.04).repeatedly());
    /*driver.a().onTrue(climber.goDown());
    driver.b().onTrue(climber.goUp());

    driver.x().onTrue(led.thingy());
    driver.y().onTrue(led.thingy2());*/
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> {
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
            }));

    /* driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    driver
        .pov(0)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driver
        .pov(180)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    configureSysidBindings();
    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));*/
  }

  private void configureSysidBindings() {
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver
        .back()
        .and(driver.y())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driver
        .back()
        .and(driver.x())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    driver
        .start()
        .and(driver.y())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driver
        .start()
        .and(driver.x())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  private static double deadband(double val) {
    return MathUtil.applyDeadband(val, STICK_DEADBAND);
  }
}
