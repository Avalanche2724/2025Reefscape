package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
  public Vision vision = new Vision();
  // public Vision vision;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  private boolean hasAppliedOperatorPerspectiveYet = false;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    // uncomment to set to coast
    // i don't know why I had this in the first place?
    // configNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public AutoFactory createAutoFactory() {
    return new AutoFactory(
        () -> getState().Pose,
        this::resetPose,
        this::followPath,
        true,
        this,
        (sample, isStart) -> {});
  }

  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  private final PIDController m_pathXController = new PIDController(10, 0, 0);
  private final PIDController m_pathYController = new PIDController(10, 0, 0);
  private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

  public void followPath(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = getState().Pose;

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!hasAppliedOperatorPerspectiveYet || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspectiveYet = true;
              });
    }

    // Do vision
    correctFromVision(vision.camera1);
    // correctFromVision(vision.camera2);
  }

  public final StructPublisher<Pose2d> drivePose =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getStructTopic("VisionPose", Pose2d.struct)
          .publish();

  public void correctFromVision(Vision.Camera camera) {
    var state = getState();

    camera.getEstimatedGlobalPose(
        -Utils.fpgaToCurrentTime(-state.Timestamp),
        getState().Pose.getRotation(),
        (est, standardDeviations) -> {
          if (est.isPresent()) {
            var estimation = est.get();
            // TODO cleanup
            drivePose.set(estimation.estimatedPose.toPose2d());
            // SmartDashboard.putData(estimation.estimatedPose.toPose2d());
            // System.out.println("Adding vision measurement");
            // System.out.println(estimation.estimatedPose.toPose2d().toString());
            addVisionMeasurement(
                estimation.estimatedPose.toPose2d(),
                Utils.fpgaToCurrentTime(estimation.timestampSeconds),
                standardDeviations);
          }
        });
  }

  private final SwerveRequest.ApplyFieldSpeeds pathPidToPoint =
      new SwerveRequest.ApplyFieldSpeeds();

  /** Command to PID to a position; useful for auto-align */
  private void pidToPosition(Pose2d target) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = getState().Pose;

    var speeds =
        new ChassisSpeeds(
            m_pathXController.calculate(pose.getX(), target.getX()),
            m_pathYController.calculate(pose.getY(), target.getY()),
            m_pathThetaController.calculate(
                pose.getRotation().getRadians(), target.getRotation().getRadians()));

    setControl(pathPidToPoint.withSpeeds(speeds));
  }

  public Command driveToPosition(Supplier<Pose2d> target) {
    return run(() -> pidToPosition(target.get()));
  }

  /**
   * Wheel radius characterization inspired by 6328: by rotating the wheels and comparing the
   * distance they travel to the angle given by the gyroscope, this command can calculate the radius
   * of the wheels!
   *
   * @return Command to run
   */
  public Command wheelCharacterization() {
    var rotateRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withRotationalRate(0.4);
    var directions = new double[4];
    var initialWheelPositions = new Distance[4];
    var initialYaw = Rotation.mutable(0);
    return sequence(
        // Start with a bit of rotation to make sure the wheels are in position:
        applyRequest(() -> rotateRequest).withTimeout(2),
        // Record initial data:
        runOnce(
            () -> {
              for (int i = 0; i < 4; i++) {
                var module = getModule(i);
                directions[i] = Math.copySign(1, module.getCurrentState().speedMetersPerSecond);
                initialWheelPositions[i] = Meters.of(module.getPosition(false).distanceMeters);
              }
              initialYaw.mut_replace(getPigeon2().getYaw().getValue());
            }),
        parallel(
            applyRequest(() -> rotateRequest),
            Commands.run(
                () -> {
                  // Find the difference in yaw since start
                  var currentYaw = getPigeon2().getYaw().getValue();
                  var gyroYawDifference = currentYaw.minus(initialYaw);
                  SmartDashboard.putNumber(
                      "Wheel characterization gyro difference", gyroYawDifference.in(Radians));

                  // Find how much the wheels have moved
                  var avgWheelMovement = Meters.mutable(0);
                  for (int i = 0; i < 4; i++) {
                    var module = getModule(i);
                    var wheelMovement =
                        (Meters.of(module.getPosition(false).distanceMeters)
                                .minus(initialWheelPositions[i]))
                            .times(directions[i]);
                    avgWheelMovement.mut_plus(wheelMovement);
                  }
                  avgWheelMovement.mut_divide(4);
                  SmartDashboard.putNumber(
                      "Wheel characterization wheel movement", avgWheelMovement.in(Meters));
                  // Find wheel circumference
                  var currentWheelCircumference = TunerConstants.kWheelRadius.times(2 * Math.PI);
                  // Based on wheel circumference, convert wheel movement in meters to rotations
                  var avgWheelMovementAngle =
                      avgWheelMovement.div(currentWheelCircumference).times(Rotation.one());
                  SmartDashboard.putNumber(
                      "Wheel characterization wheel movement radians",
                      avgWheelMovementAngle.in(Radians));

                  // Find the drive base radius of the wheels
                  var drivebaseRadius = Meters.of(getModuleLocations()[0].getNorm());
                  // Find the circumference from this radius
                  var drivebaseCircumference = drivebaseRadius.times(2 * Math.PI);
                  // Find the arc length that was actually traveled by each wheel based on gyro
                  var arcLength =
                      drivebaseCircumference.times(gyroYawDifference.div(Rotation.one()));
                  SmartDashboard.putNumber(
                      "Wheel characterization arc traveled", arcLength.in(Meters));

                  // Find what the wheel circumference should be based on the arc length
                  var actualWheelCircumference =
                      arcLength.div(avgWheelMovementAngle.div(Rotation.one()));
                  // Find what the wheel radius should be
                  var calculatedRadius = actualWheelCircumference.div(2 * Math.PI);
                  // Put to dashboard
                  SmartDashboard.putNumber(
                      "Wheel characterization CALCULATED RADIUS", calculatedRadius.in(Inches));
                })));
  }

  // SysId

  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null,
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7), // Use dynamic voltage of 7 V
              null,
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

  // Simulation:
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());

              // Camera simulation
              if (vision != null) {
                var debugField = vision.getSimDebugField();
                debugField.getObject("EstimatedRobot").setPose(getState().Pose);
                vision.simulationPeriodic(getState().Pose);
              }
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
