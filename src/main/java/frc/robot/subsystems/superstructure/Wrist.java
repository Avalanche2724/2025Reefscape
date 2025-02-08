package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Wrist {
  // Constants
  private static final int WRIST_ID = 51;
  public static final double GEAR_RATIO = 42.18;
  public static final double MASS = Kilograms.convertFrom(15, Pounds);
  public static final double ARM_LEN = Meters.convertFrom(30, Inches);
  public static final double ARM_OFFSET_DEG = -9.0;
  public static final double ARM_OFFSET = Rotations.convertFrom(ARM_OFFSET_DEG, Degrees);
  public static final double UP_LIMIT = Rotations.convertFrom(90 + ARM_OFFSET_DEG, Degrees);
  public static final double DOWN_LIMIT = Rotations.convertFrom(-90 + ARM_OFFSET_DEG, Degrees);

  private final TalonFX motor = new TalonFX(WRIST_ID);
  private final MotionMagicVoltage control = new MotionMagicVoltage(0);

  public Wrist() {
    var config = new TalonFXConfiguration();
    // values from sim, fix later
    config.Slot0.kP = 59.86;
    config.Slot0.kD = 7.1244;
    config.Slot0.kS = 0.00235;
    config.Slot0.kV = 5.2052;
    config.Slot0.kA = 0.2534;
    config.Slot0.kG = 0.77609;

    config.MotionMagic.MotionMagicAcceleration = 6; // rotations per second squared
    config.MotionMagic.MotionMagicCruiseVelocity = 2; // rotations per second
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    motor.setPosition(ARM_OFFSET);
  }

  private void setMotorRotations(double pos) {
    motor.setControl(control.withPosition(pos));
  }

  void setMotorDegreesOffset(double deg) {
    setMotorRotations(Rotations.convertFrom(deg + ARM_OFFSET_DEG, Degrees));
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
            Meters.convertFrom(1.5, Inch),
            -90,
            0,
            new Color8Bit(Color.kRed));
    wristRotatePart =
        wristMechanism.append(
            new MechanismLigament2d(
                "wrist_rotate_part_2",
                Meters.convertFrom(11.5, Inches),
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
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("arm_sysid", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
              null,
              Superstructure.instance));
}
