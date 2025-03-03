package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class Wrist {
  // Constants
  private static final int WRIST_ID = 51;
  private static final int WRIST_ENCODER_ID = 52;
  public static final double GEAR_RATIO = 64;
  public static final double MASS = Kilograms.convertFrom(15, Pounds);
  public static final double ARM_LEN = Meters.convertFrom(30, Inches);
  // Offset we need to subtract from motor to get the actual position (defined as 0=horizontal arm)
  // This is because 0 in motor should be max gravity but the center of gravity is off center
  public static final double ARM_OFFSET_DEG = Degrees.convertFrom(-0.049, Rotations);
  public static final double ARM_OFFSET = Rotations.convertFrom(ARM_OFFSET_DEG, Degrees);
  public static final double UP_LIMIT = Rotations.convertFrom(90 + ARM_OFFSET_DEG, Degrees);
  public static final double DOWN_LIMIT = Rotations.convertFrom(-90 + ARM_OFFSET_DEG, Degrees);

  private final TalonFX motor = new TalonFX(WRIST_ID);

  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withSlot(0);
  private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(1);

  public final SparkMax absoluteEncoderSparkMax =
      new SparkMax(WRIST_ENCODER_ID, MotorType.kBrushed);
  public final SparkAbsoluteEncoder absoluteEncoder = absoluteEncoderSparkMax.getAbsoluteEncoder();

  double positionSwitchThreshold = 0.01;

  public Wrist() {
    var config = new TalonFXConfiguration();

    config.Slot0.kP = 75;
    config.Slot0.kD = 10;
    config.Slot0.kS = (0.48 - 0.37) / 2;
    config.Slot0.kV = 7.9347;
    config.Slot0.kA = 0.13794;
    config.Slot0.kG = (0.48 + 0.37) / 2;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot1.kP = 22; // todo: try retuning and lowering?
    config.Slot1.kD = 18;
    config.Slot1.kS = config.Slot0.kS;
    config.Slot1.kV = config.Slot0.kV;
    config.Slot1.kA = config.Slot0.kA;
    config.Slot1.kG = config.Slot0.kG;
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    config.MotionMagic.MotionMagicAcceleration = 0.5; // rotations per second squared
    config.MotionMagic.MotionMagicCruiseVelocity = 1; // rotations per second
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = UP_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DOWN_LIMIT;

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
    sparkMaxConfig.absoluteEncoder.zeroCentered(true).zeroOffset(0.217).inverted(false);
    sparkMaxConfig.signals.absoluteEncoderPositionPeriodMs(1);
    sparkMaxConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    absoluteEncoderSparkMax.configure(
        sparkMaxConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    Robot.instance.addPeriodic(absoluteEncoderResetter(), 0.002);
  }

  private double absoluteEncoderPosition() {
    return Robot.isReal()
        ? absoluteEncoder.getPosition()
        : Rotations.convertFrom(armSim.getAngleRads(), Radians);
  }

  // this is probably a horrible idea and I apologize to anybody looking at this in the future
  private Runnable absoluteEncoderResetter() {
    var setter =
        new TalonFXConfigurator(new DeviceIdentifier(WRIST_ID, "talon fx", "")) {
          @Override
          protected void reportIfFrequent() {}
        };
    return () -> setter.setPosition(absoluteEncoderPosition() + ARM_OFFSET, 0);
  }

  private double lastPositionSet = 0;
  private boolean setPosition = false;

  private void setMotorRotations(double pos) {
    setPosition = true;
    lastPositionSet = pos;

    if (Math.abs(motor.getPosition().getValueAsDouble() - pos) < positionSwitchThreshold) {
      motor.setControl(positionControl.withPosition(lastPositionSet));
    } else {
      motor.setControl(motionMagicControl.withPosition(pos));
    }
  }

  void setMotorDegreesOffset(double deg) {
    setMotorRotations(Rotations.convertFrom(deg + ARM_OFFSET_DEG, Degrees));
  }

  void setMotorDutyCycle(double d) {
    setPosition = false;
    motor.set(d);
  }

  public double getWristRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getWristDegrees() {
    return Degrees.convertFrom(getWristRotations(), Rotations);
  }

  public double getWristDegreesOffset() {
    return getWristDegrees() - ARM_OFFSET_DEG;
  }

  // Mechanism2d:
  MechanismLigament2d wristRotatePart;
  MechanismLigament2d wristEnd;

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
    wristEnd =
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

  // Simulation
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LEN, MASS),
          ARM_LEN,
          Radians.convertFrom(DOWN_LIMIT, Rotations),
          Radians.convertFrom(UP_LIMIT, Rotations),
          true,
          Radians.convertFrom(ARM_OFFSET, Rotations));

  public void simulationPeriodic(double deltaTime) {
    var motorSim = motor.getSimState();
    armSim.setInput(motorSim.getMotorVoltage());
    armSim.update(deltaTime);

    motorSim.setRotorVelocity(
        RotationsPerSecond.convertFrom(armSim.getVelocityRadPerSec(), RadiansPerSecond)
            * GEAR_RATIO);

    motorSim.setRawRotorPosition(
        Rotations.convertFrom(armSim.getAngleRads(), Radians) * GEAR_RATIO);
  }

  // SysId
  public VoltageOut sysIdControl = new VoltageOut(0);

  public SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.1).div(Second.one()),
              Volts.of(1),
              null,
              (state) -> SignalLogger.writeString("arm_sysid", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
              null,
              Superstructure.instance));

  public void periodic() {
    // TODO add telemetry
    if (setPosition
        && Math.abs(motor.getPosition().getValueAsDouble() - lastPositionSet)
            < positionSwitchThreshold) {
      motor.setControl(positionControl.withPosition(lastPositionSet));
    }
  }
}
