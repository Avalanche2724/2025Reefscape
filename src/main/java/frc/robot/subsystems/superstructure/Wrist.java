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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class Wrist {
  // Constants
  public static final int WRIST_ID = 51;
  public static final int WRIST_ENCODER_ID = 52;
  public static final double GEAR_RATIO = 64;

  public static final double ZERO_OFFSET = 0.206; // rotations
  public static final double ARM_OFFSET = Robot.isSimulation() ? 0 : -0.049; // for kG adjustment
  public static final double UP_LIMIT = Rotations.convertFrom(90, Degrees);
  public static final double DOWN_LIMIT = Rotations.convertFrom(-10, Degrees);

  private static final double ENCODER_POSITION_RESET_SEC = 0.02;
  // I/O
  private final TalonFX motor = new TalonFX(WRIST_ID);
  // Signals
  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent();
  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
  // Controls
  // private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withSlot(0);
  // private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);

  // Absolute encoder reading
  private final SparkMax absoluteEncoderSparkMax =
      new SparkMax(WRIST_ENCODER_ID, MotorType.kBrushed);
  private final SparkAbsoluteEncoder absoluteEncoder = absoluteEncoderSparkMax.getAbsoluteEncoder();
  // Simulation
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO, // Following arm length/mass are estimates for simulation
          SingleJointedArmSim.estimateMOI(
              Meters.convertFrom(39, Inches), Kilograms.convertFrom(15, Pounds)),
          Kilograms.convertFrom(15, Pounds),
          Radians.convertFrom(DOWN_LIMIT, Rotations),
          Radians.convertFrom(UP_LIMIT, Rotations),
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

  // Mechanism2d:
  private MechanismLigament2d wristRotatePart;
  private double lastPositionSet = 0;
  private boolean setPosition = false;

  private final double wrist_pid_sec = 0.005;

  private final PIDController pid = new PIDController(40, 0, 0.4, wrist_pid_sec);
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          (0.5 - 0.37) / 2,
          (0.5 + 0.37) / 2,
          1 / Radians.convertFrom(1 / 7.94, Rotations),
          1 / Radians.convertFrom(1 / 0.14, Rotations),
          wrist_pid_sec);

  public Wrist() {
    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(config);

    // for phoenix tuner x tuning:
    motor.getClosedLoopError().setUpdateFrequency(50);
    motor.getClosedLoopReference().setUpdateFrequency(50);
    motor.getClosedLoopDerivativeOutput().setUpdateFrequency(50);
    motor.getClosedLoopOutput().setUpdateFrequency(50);
    motor.getClosedLoopProportionalOutput().setUpdateFrequency(50);
    motor.getClosedLoopFeedForward().setUpdateFrequency(50);

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.absoluteEncoder.zeroCentered(true).zeroOffset(ZERO_OFFSET).inverted(false);
    sparkMaxConfig.signals.absoluteEncoderPositionPeriodMs(1);
    sparkMaxConfig.signals.absoluteEncoderVelocityPeriodMs(1);
    sparkMaxConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    absoluteEncoderSparkMax.configure(
        sparkMaxConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    Robot.instance.addPeriodic(absoluteEncoderResetter(), ENCODER_POSITION_RESET_SEC);

    var pidControlNotifier = new Notifier(this::pidcontrol);
    pidControlNotifier.startPeriodic(wrist_pid_sec);
  }

  // Trapezoid profile
  final TrapezoidProfile m_profile_decel =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.4, 0.5));
  final TrapezoidProfile m_profile_accel =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.4, 1.4));

  TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(0, 0);

  public void periodic() {
    motorPosition.refresh();
    motorVelocity.refresh();
    motorTorqueCurrent.refresh();
    motorVoltage.refresh();
    // Log to NetworkTables
    SmartDashboard.putNumber("Wrist Absolute Encoder Pos", getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Wrist Degrees", getWristDegrees());
    SmartDashboard.putNumber("Wrist Rotations", getWristRotations());
    if (Robot.isSimulation())
      SmartDashboard.putNumber(
          "Wrist Sim Rotations (no offset)", Rotations.convertFrom(armSim.getAngleRads(), Radians));

    SmartDashboard.putNumber("Wrist Velocity", getVelocity());
    SmartDashboard.putNumber("Wrist Current", getTorqueCurrent());
    SmartDashboard.putNumber("Wrist Voltage", getVoltage());
    SmartDashboard.putNumber("Wrist ultimate target no offset", lastPositionSet);
  }

  private void pidcontrol() {
    if (setPosition) {
      var last_setpoint = m_setpoint;
      var chosen_profile = m_profile_decel;
      m_setpoint = chosen_profile.calculate(wrist_pid_sec, last_setpoint, m_goal);

      double velocityDiffSign = m_setpoint.velocity - last_setpoint.velocity;
      double positionDiffSign = m_setpoint.position - last_setpoint.position;

      // todo later

      if (positionDiffSign < 0) {
        if (velocityDiffSign < 0) {
          chosen_profile = m_profile_accel;
        }
      } else {
        if (velocityDiffSign > 0) {
          chosen_profile = m_profile_accel;
        }
      }
      if (chosen_profile == m_profile_accel) {
        m_setpoint = chosen_profile.calculate(wrist_pid_sec, last_setpoint, m_goal);
      }

      double voltageFeedforward =
          feedforward.calculateWithVelocities(
              Radians.convertFrom(m_setpoint.position + ARM_OFFSET, Rotations),
              Radians.convertFrom(last_setpoint.velocity, Rotations),
              Radians.convertFrom(m_setpoint.velocity, Rotations));

      double position = getAbsoluteEncoderPosition();
      double pidOutput = pid.calculate(position, m_setpoint.position);

      SmartDashboard.putNumber("pid current pos", position);
      SmartDashboard.putNumber("pid setpoint", m_setpoint.position);
      SmartDashboard.putNumber("pid setpoint velocity", m_setpoint.velocity);
      SmartDashboard.putNumber("feedforward", voltageFeedforward);
      SmartDashboard.putNumber("pid output", pidOutput);

      motor.setControl(voltageControl.withOutput(pidOutput + voltageFeedforward));
    }
  }

  // Signals

  public double getAbsoluteEncoderPosition() {
    if (Robot.isSimulation()) return Rotations.convertFrom(armSim.getAngleRads(), Radians);
    return absoluteEncoder.getPosition();
  }

  public double getWristRotations() {
    return getAbsoluteEncoderPosition();
  }

  public double getWristDegrees() {
    return Degrees.convertFrom(getWristRotations(), Rotations);
  }

  // do not use may be bad
  private double getVelocity() {
    return motorVelocity.getValueAsDouble();
  }

  public double getTorqueCurrent() {
    return motorTorqueCurrent.getValueAsDouble();
  }

  public double getVoltage() {
    return motorVoltage.getValueAsDouble();
  }

  // this is probably a horrible idea and I apologize to anybody looking at this in the future
  private Runnable absoluteEncoderResetter() {
    var setter =
        new TalonFXConfigurator(new DeviceIdentifier(WRIST_ID, "talon fx", "")) {
          @Override
          protected void reportIfFrequent() {}
        };

    return () -> {
      if (Robot.isSimulation())
        return; // Causes weird conflict with simulationPeriodic setRawRotorPosition
      double pos = getAbsoluteEncoderPosition();
      setter.setPosition(pos, 0);
    };
  }

  // Controls and stuff

  private void setMotorRotations(double pos) {
    if (!setPosition) {
      m_setpoint =
          new TrapezoidProfile.State(getWristRotations(), motorVelocity.getValueAsDouble());
    }
    setPosition = true;
    lastPositionSet = pos;
    m_goal = new TrapezoidProfile.State(lastPositionSet, 0);
    // let periodic do the pid thingy
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
