package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Util;

public class Wrist {
  // Constants
  public static final int WRIST_ID = 51;
  public static final int WRIST_ENCODER_ID = 52;
  public static final double GEAR_RATIO = 64;
  public static final double ZERO_OFFSET = 0.206; // rotations
  // adjustment on position passed to feedforward for kG accuracy
  public static final double ARM_OFFSET = Robot.isSimulation() ? 0 : -0.049;

  // limits
  private static final double UP_LIMIT = Rotations.convertFrom(95, Degrees);
  private static final double DOWN_LIMIT = Rotations.convertFrom(-15, Degrees);

  // PID stuff
  private final double WRIST_PID_PERIOD = 0.02;
  private final PIDController pid =
      new PIDController(Robot.isReal() ? 12 : 60, 0, 0.5, WRIST_PID_PERIOD);
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          (0.5 - 0.37) / 2,
          (0.5 + 0.37) / 2,
          1 / Radians.convertFrom(1 / 7.94, Rotations),
          1 / Radians.convertFrom(1 / 0.14, Rotations),
          WRIST_PID_PERIOD);
  // Trapezoid profile
  private final TrapezoidProfile profileDeceleration =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 0.45));
  private final TrapezoidProfile profileAcceleration =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 1.05));
  // I/O
  private final TalonFX motor = new TalonFX(WRIST_ID);
  // Signals
  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent();
  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
  // Controls
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
  // Absolute encoder reading
  private final SparkMax absoluteEncoderSparkMax =
      new SparkMax(WRIST_ENCODER_ID, MotorType.kBrushed);
  private final SparkAbsoluteEncoder absoluteEncoder = absoluteEncoderSparkMax.getAbsoluteEncoder();
  private final Runnable absoluteEncoderSetter = absoluteEncoderSetter();
  // Simulation
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO, // Following arm length/mass are estimates for simulation
          SingleJointedArmSim.estimateMOI(
              Meters.convertFrom(39, Inches), Kilograms.convertFrom(15, Pounds)),
          Kilograms.convertFrom(15, Pounds),
          Radians.convertFrom(Rotations.convertFrom(-10, Degrees), Rotations),
          Radians.convertFrom(Rotations.convertFrom(90, Degrees), Rotations),
          true,
          0);
  // SysId
  public VoltageOut sysIdControl = new VoltageOut(0);

  @SuppressWarnings("unused")
  public SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.2).div(Second.one()),
              Volts.of(2),
              null,
              (state) -> SignalLogger.writeString("Arm_SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
              null,
              Superstructure.instance));

  // Other PID stuff:
  private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State(0, 0);
  // Whether PID control should run
  private boolean setPosition = false;

  // Mechanism2d:
  private MechanismLigament2d wristRotatePart;
  private double lastPositionSet = 0;

  public Wrist() {
    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Note: TalonFX position/feedback is not actually used in code, but if
    // the encoder disconnects for some reason, the limits will stop the wrist from going too far
    // TODO: have it be used for pid or something
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = UP_LIMIT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DOWN_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    motor.getConfigurator().apply(config);

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.absoluteEncoder.zeroCentered(true).zeroOffset(ZERO_OFFSET).inverted(false);
    sparkMaxConfig.signals.absoluteEncoderPositionPeriodMs(1);
    sparkMaxConfig.signals.absoluteEncoderVelocityPeriodMs(1);
    sparkMaxConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    absoluteEncoderSparkMax.configure(
        sparkMaxConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // SmartDashboard.putData("wrist pid", pid);
  }

  public void periodic() {
    motorPosition.refresh();
    motorVelocity.refresh();
    motorTorqueCurrent.refresh();
    motorVoltage.refresh();

    absoluteEncoderSetter.run();
    runPositionControl();

    Util.logDouble("Wrist Pos", getAbsoluteEncoderPosition());
    Util.logDouble("Wrist Degrees", getWristDegrees());
    Util.logDouble("Wrist Velocity", getVelocity());
    Util.logDouble("Wrist Current", getTorqueCurrent());
    Util.logDouble("Wrist Voltage", getVoltage());
    Util.logDouble("Wrist Target", lastPositionSet);
  }

  private void runPositionControl() {
    if (setPosition) {
      var lastSetpoint = profileSetpoint;
      var chosenProfile = profileDeceleration;
      profileSetpoint = chosenProfile.calculate(WRIST_PID_PERIOD, lastSetpoint, profileGoal);

      double velocityDiffSign = profileSetpoint.velocity - lastSetpoint.velocity;
      double positionDiffSign = profileSetpoint.position - lastSetpoint.position;

      if (positionDiffSign < 0) {
        if (velocityDiffSign < 0) {
          chosenProfile = profileAcceleration;
        }
      } else {
        if (velocityDiffSign > 0) {
          chosenProfile = profileAcceleration;
        }
      }
      if (chosenProfile == profileAcceleration) {
        profileSetpoint = chosenProfile.calculate(WRIST_PID_PERIOD, lastSetpoint, profileGoal);
      }

      double voltageFeedforward =
          feedforward.calculateWithVelocities(
              Radians.convertFrom(profileSetpoint.position + ARM_OFFSET, Rotations),
              Radians.convertFrom(lastSetpoint.velocity, Rotations),
              Radians.convertFrom(profileSetpoint.velocity, Rotations));

      double position = getAbsoluteEncoderPosition();
      double pidOutput = pid.calculate(position, profileSetpoint.position);

      Util.logDouble("Wrist pid setpoint", profileSetpoint.position);
      Util.logDouble("Wrist pid setpoint velocity", profileSetpoint.velocity);
      Util.logDouble("Wrist pid feedforward", voltageFeedforward);
      Util.logDouble("Wrist pid output", pidOutput);

      motor.setControl(voltageControl.withOutput(pidOutput + voltageFeedforward));
    }
  }

  // Signals

  public double getAbsoluteEncoderPosition() {
    var pos = absoluteEncoder.getPosition();
    if (Robot.isSimulation()) return Rotations.convertFrom(armSim.getAngleRads(), Radians);

    // TODO alerting better
    if (absoluteEncoderSparkMax.getFaults().sensor) {
      System.out.println("Warning: sensor fault");
    }
    if (pos == 0) {
      System.out.println("Warning: pos exactly 0, disconnected?");
    }

    return absoluteEncoder.getPosition();
  }

  public double getWristRotations() {
    return getAbsoluteEncoderPosition();
  }

  public double getWristDegrees() {
    return Degrees.convertFrom(getWristRotations(), Rotations);
  }

  // NOTE: uses motor velocity, might be inaccurate
  private double getVelocity() {
    return motorVelocity.getValueAsDouble();
  }

  public double getTorqueCurrent() {
    return motorTorqueCurrent.getValueAsDouble();
  }

  public double getVoltage() {
    return motorVoltage.getValueAsDouble();
  }

  // this is probably a bad idea and I apologize to anybody looking at this in the future
  private Runnable absoluteEncoderSetter() {
    var setter =
        new TalonFXConfigurator(new DeviceIdentifier(WRIST_ID, "talon fx", "")) {
          @Override
          protected void reportIfFrequent() {}
        };

    return () -> {
      if (Robot.isSimulation())
        return; // Causes weird conflict with simulationPeriodic setRawRotorPosition
      double pos = getAbsoluteEncoderPosition();
      if (absoluteEncoderSparkMax.getFaults().sensor || pos == 0) {
        return;
      }
      setter.setPosition(pos, 0);
    };
  }

  // Controls and stuff

  private void setMotorRotations(double pos) {
    if (!setPosition) {
      profileSetpoint =
          new TrapezoidProfile.State(getWristRotations(), motorVelocity.getValueAsDouble());
    }
    setPosition = true;
    lastPositionSet = pos;
    profileGoal = new TrapezoidProfile.State(lastPositionSet, 0);
    // don't command motor from here, but let periodic do pid control
  }

  void setMotorDegrees(double deg) {
    setMotorRotations(Rotations.convertFrom(deg, Degrees));
  }

  void stopMotor() {
    setPosition = false;
    motor.set(0);
  }

  // Mechanism + simulation
  public MechanismLigament2d createMechanism2d() {
    var wristMechanism =
        new MechanismLigament2d(
            "wrist_base_elev_part_1",
            Meters.convertFrom(2, Inch),
            -90,
            0,
            new Color8Bit(Color.kRed));
    wristRotatePart =
        wristMechanism.append(
            new MechanismLigament2d(
                "wrist_rotate_part_2",
                Meters.convertFrom(11, Inches),
                getWristDegrees(),
                Centimeters.convertFrom(1, Inch),
                new Color8Bit(Color.kPurple)));
    var wristRotatePart2 =
        wristRotatePart.append(
            new MechanismLigament2d(
                "wrist_rotate_part_3",
                Meters.convertFrom(4, Inches),
                90,
                Centimeters.convertFrom(1, Inch),
                new Color8Bit(Color.kPurple)));
    wristRotatePart2.append(
        new MechanismLigament2d(
            "wrist_end_part_4",
            Meters.convertFrom(13, Inches),
            -90,
            Centimeters.convertFrom(4, Inches),
            new Color8Bit(Color.kBlue)));
    return wristMechanism;
  }

  public void updateMechanism2d() {
    wristRotatePart.setAngle(getWristDegrees());
  }

  public void simulationPeriodic(double deltaTime) {
    var motorSim = motor.getSimState();

    armSim.setInput(-motorSim.getMotorVoltage());
    armSim.update(deltaTime);

    motorSim.setRotorVelocity(
        -RotationsPerSecond.convertFrom(armSim.getVelocityRadPerSec(), RadiansPerSecond)
            * GEAR_RATIO);

    motorSim.setRawRotorPosition(
        -Rotations.convertFrom(armSim.getAngleRads(), Radians) * GEAR_RATIO);
  }
}
