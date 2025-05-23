package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
  // Swerve requests for auto
  private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  private final PIDController pathXController = new PIDController(9, 0, 0);
  private final PIDController pathYController = new PIDController(9, 0, 0);
  private final PIDController pathThetaController = new PIDController(7, 0, 0);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** PID controllers for auto-align */

  // private final PIDController pathXController = new PIDController(10, 0, 0);
  //  private final PIDController pathYController = new PIDController(10, 0, 0);

  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();

  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null,
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));
  /* The SysId routine to test */
  public final SysIdRoutine m_sysIdRoutineToApply = sysIdRoutineTranslation;
  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4), // Use dynamic voltage of 4 V
              null,
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine sysIdRoutineRotation =
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
  public Vision vision = new Vision();
  double[] m_poseArray = new double[3];
  private boolean hasAppliedOperatorPerspectiveYet = false;
  private Notifier visionNotifier = null;
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // Auto align stuff

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

    createVisionThread();

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

  public Command brakeOnce() {
    return runOnce(() -> setControl(brake));
  }

  // Vision

  public AutoFactory createAutoFactory() {
    return new AutoFactory(() -> getState().Pose, this::resetPose, this::followPath, true, this);
  }

  public void logPose2d(String key, Pose2d pose) {
    m_poseArray[0] = pose.getX();
    m_poseArray[1] = pose.getY();
    m_poseArray[2] = pose.getRotation().getDegrees();
    SignalLogger.writeDoubleArray(key, m_poseArray);
  }

  // Commands for auto-align

  public void followPath(SwerveSample sample) {
    pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = getState().Pose;

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    logPose2d("Auto/CurrentPose", pose);
    logPose2d("Auto/TargetPose", sample.getPose());

    setControl(
        pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  public void createVisionThread() {
    visionNotifier =
        new Notifier(
            () -> {
              Threads.setCurrentThreadPriority(false, 0);

              // Do vision
              correctFromVision(vision.cameraFrElev);
              correctFromVision(vision.cameraFlSwerve);
            });
    visionNotifier.startPeriodic(0.01);
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
                    allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
                hasAppliedOperatorPerspectiveYet = true;
              });
    }
  }

  public void integrateVisionCorrection(
      EstimatedRobotPose estimation, Matrix<N3, N1> standardDeviations) {
    var pose2d = estimation.estimatedPose.toPose2d();
    logPose2d("Vision_Estimation", pose2d);
    // TODO log

    addVisionMeasurement(
        pose2d, Utils.fpgaToCurrentTime(estimation.timestampSeconds), standardDeviations);
  }

  public void correctFromVision(Vision.Camera camera) {
    camera.getEstimatedGlobalPose(
        -Utils.fpgaToCurrentTime(-getState().Timestamp),
        getState().Pose.getRotation(),
        this::integrateVisionCorrection);
  }

  // auto-align code mostly adapted from Ben from CTRE's messages in the FRC discord
  // but kinda messy, should fix later lol
  public static class PidToPointRequest implements SwerveRequest {
    public Pose2d target;

    private final PIDController autoAlignXController = new PIDController(9, 0, 0.2);
    private final PIDController autoAlignYController = new PIDController(9, 0, 0.2);
    private final PIDController autoAlignThetaController = new PIDController(7, 0, 0);

    private final TrapezoidProfile autoAlignProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 3.7));
    private TrapezoidProfile.State autoAlignState = new TrapezoidProfile.State();
    private TrapezoidProfile.State autoAlignGoal;
    private Rotation2d autoAlignHeading;
    private Pose2d autoAlignInitialPose;
    private final SwerveRequest.ApplyFieldSpeeds pathPidToPoint =
        new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    @Override
    public StatusCode apply(
        SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
      autoAlignThetaController.enableContinuousInput(-Math.PI, Math.PI);
      // calculate our new distance and velocity in the desired direction of travel
      autoAlignState = autoAlignProfile.calculate(1.0 / 250.0, autoAlignState, autoAlignGoal);

      Pose2d curPose = parameters.currentPose;

      double currentDistance = curPose.getTranslation().getDistance(target.getTranslation());

      double ffMinRadius = 0.2;
      double ffMaxRadius = 1.1;

      double ffScaler =
          MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

      double xSpeed =
          ffScaler * autoAlignState.velocity * autoAlignHeading.getCos()
              + autoAlignXController.calculate(
                  curPose.getX(),
                  autoAlignInitialPose.getX()
                      + autoAlignState.position * autoAlignHeading.getCos());
      double ySpeed =
          ffScaler * autoAlignState.velocity * autoAlignHeading.getSin()
              + autoAlignYController.calculate(
                  curPose.getY(),
                  autoAlignInitialPose.getY()
                      + autoAlignState.position * autoAlignHeading.getSin());
      double thetaSpeed =
          autoAlignThetaController.calculate(
              curPose.getRotation().getRadians(), target.getRotation().getRadians());
      if (Math.abs(
              MathUtil.angleModulus(
                  curPose.getRotation().getRadians() - target.getRotation().getRadians()))
          < Radians.convertFrom(0.7, Degrees)) {
        thetaSpeed = 0;
      }

      if (currentDistance < Meters.convertFrom(0.8, Inches)) {
        ySpeed = 0;
        xSpeed = 0;
      }

      return pathPidToPoint
          .withSpeeds(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed))
          .apply(parameters, modulesToApply);
    }

    public void initializeRequest(Pose2d target, SwerveDriveState state) {
      this.target = target;
      autoAlignInitialPose = state.Pose;

      var translation = target.getTranslation().minus(autoAlignInitialPose.getTranslation());
      var distance = translation.getNorm();
      autoAlignHeading = translation.getAngle();

      var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
      var initialVel =
          speeds.vxMetersPerSecond * autoAlignHeading.getCos()
              + speeds.vyMetersPerSecond * autoAlignHeading.getSin();

      autoAlignState = new TrapezoidProfile.State(0, initialVel);
      autoAlignGoal = new TrapezoidProfile.State(distance, 0);
    }
  }

  public Command driveToPosition(Supplier<Pose2d> target) {
    var req = new PidToPointRequest();

    return runOnce(
            () -> {
              // inside method
              req.initializeRequest(target.get(), getState());
            })
        .andThen(run(() -> setControl(req)).finallyDo(() -> setControl(brake)));
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

  // Simulation stuff

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
    m_simNotifier.startPeriodic(0.005);
  }
}
