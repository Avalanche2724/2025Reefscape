package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
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

  /*
  A brief explanation of wrist positioning:
  There is the value the absolute encoder gets (from 0-1); the spark max subtracts 0.5 (making it -0.5 to 0.5)
  and then applies the zero offset, wrapping around, where zero is "arm plate horizontal".
  Then we add ARM_OFFSET before sending the position to the talonfx motor, because it needs to apply the correct
  kG value to counteract gravity, and the arm center of gravity is off-center from the arm horizontal position.
  (Which probably results in a difference of 0.13 volts at most; it should be slightly impactful)
   */
  public static final double ZERO_OFFSET = 0.206; // rotations
  public static final double ARM_OFFSET = -0.049;
  public static final double ARM_OFFSET_DEG = Degrees.convertFrom(ARM_OFFSET, Rotations);
  public static final double UP_LIMIT = Rotations.convertFrom(90, Degrees);
  public static final double DOWN_LIMIT = Rotations.convertFrom(-45, Degrees);

  private static final double THRESHOLD_SWITCHING_PID_GAINS = 0.01;
  private static final double ENCODER_POSITION_RESET_SEC = 0.001;
  // I/O
  private final TalonFX motor = new TalonFX(WRIST_ID);
  // Signals
  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent();
  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
  // Controls
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withSlot(0);
  private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(1);
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
              Meters.convertFrom(25, Inches), Kilograms.convertFrom(15, Pounds)),
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

  public Wrist() {
    var config = new TalonFXConfiguration();

    // config.Slot0.kP = Robot.isSimulation() ? 200 : 90;
    config.Slot0.kP = 63.413;
    // config.Slot0.kD = 8;
    // config.Slot0.kD = 0.22525;
    config.Slot0.kD = 4;
    config.Slot0.kS = (0.48 - 0.37) / 2;
    config.Slot0.kV = 7.94;
    config.Slot0.kA = 0.14;
    config.Slot0.kG = (0.48 + 0.37) / 2;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot1.kP = 12; // todo: try retuning and lowering?
    config.Slot1.kD = 1;
    config.Slot1.kS = config.Slot0.kS;
    config.Slot1.kV = config.Slot0.kV;
    config.Slot1.kA = config.Slot0.kA;
    config.Slot1.kG = config.Slot0.kG;
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    config.MotionMagic.MotionMagicCruiseVelocity = 1.5; // rotations per second; maximum?
    config.MotionMagic.MotionMagicAcceleration = 1.5; // rotations per second squared
    config.MotionMagic.MotionMagicJerk = 3; // rotations per second cubed
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = UP_LIMIT + ARM_OFFSET;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DOWN_LIMIT + ARM_OFFSET;

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
    if (Robot.isSimulation()) {
      motor.setPosition(ARM_OFFSET);
    }
  }

  public void periodic() {
    motorPosition.refresh();
    motorVelocity.refresh();
    motorTorqueCurrent.refresh();
    motorVoltage.refresh();
    // Log to NetworkTables
    SmartDashboard.putNumber("Wrist Absolute Encoder Pos", getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Wrist Degrees (with offset)", getWristDegreesOffset());
    SmartDashboard.putNumber("Wrist Velocity", getVelocity());
    SmartDashboard.putNumber("Wrist Current", getTorqueCurrent());
    SmartDashboard.putNumber("Wrist Voltage", getVoltage());

    if (setPosition
        && Math.abs(getWristRotations() - lastPositionSet) < THRESHOLD_SWITCHING_PID_GAINS) {
      motor.setControl(positionControl.withPosition(lastPositionSet));
    }
  }

  // Signals

  public double getAbsoluteEncoderPosition() {
    return absoluteEncoder.getPosition();
  }

  public double getWristRotations() {
    return motorPosition.getValueAsDouble();
  }

  public double getWristDegrees() {
    return Degrees.convertFrom(getWristRotations(), Rotations);
  }

  public double getWristDegreesOffset() {
    return getWristDegrees() - ARM_OFFSET_DEG;
  }

  public double getVelocity() {
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
      if (Robot.isSimulation()) return; // Conflicts with simulationPeriodic setRawRotorPosition
      double pos = getAbsoluteEncoderPosition();
      double vel = absoluteEncoder.getVelocity();
      if (pos == 0 && vel == 0) {
        // TODO fix me later and implement better alerting
        DriverStation.reportError("Check absolute encoder reset", false);
      } else {
        setter.setPosition(pos + ARM_OFFSET, 0);
      }
    };
  }

  // Controls and stuff

  private void setMotorRotations(double pos) {
    setPosition = true;
    lastPositionSet = pos;

    if (Math.abs(getWristRotations() - pos) < THRESHOLD_SWITCHING_PID_GAINS) {
      motor.setControl(positionControl.withPosition(lastPositionSet));
    } else {
      motor.setControl(motionMagicControl.withPosition(pos));
    }
  }

  void setMotorDegreesOffset(double deg) {
    setMotorRotations(Rotations.convertFrom(deg + ARM_OFFSET_DEG, Degrees));
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
    wristRotatePart.setAngle(getWristDegreesOffset());
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
