package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Wrist extends SubsystemBase {
  private static final int WRIST_ID = 51;
  public static final double GEAR_RATIO = 42.18;
  public static final double MASS = Kilograms.convertFrom(15, Pounds);
  public static final double ARM_LEN = Meters.convertFrom(30, Inches);
  // angle from arm flat to center of gravity; approximated rn
  public static final double ARM_OFFSET = -9.0;
  public static final double UP_LIMIT = Rotations.convertFrom(90 + ARM_OFFSET, Degrees);
  public static final double DOWN_LIMIT = Rotations.convertFrom(-90 + ARM_OFFSET, Degrees);

  public enum WristPosition {
    BASE(0),
    SCORE(-30),
    UP(30);

    public final double rotations;

    WristPosition(double degrees) {
      this.rotations = Rotations.convertFrom(degrees + ARM_OFFSET, Degrees);
    }
  }

  private final MotionMagicVoltage control = new MotionMagicVoltage(0);

  private final TalonFX motor = new TalonFX(WRIST_ID);

  public Wrist() {
    var config = new TalonFXConfiguration();
    // values from sim, fix later
    config.Slot0.kP = 50.674;
    config.Slot0.kD = 5.5743;
    config.Slot0.kS = 0.078264;
    config.Slot0.kV = 4.8984;
    config.Slot0.kA = 0.27535;
    config.Slot0.kG = 0.76522;

    config.MotionMagic.MotionMagicAcceleration = 400;
    config.MotionMagic.MotionMagicCruiseVelocity = 75;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
    motor.setPosition(WristPosition.BASE.rotations);
  }

  double targetPos;

  private void setMotorPosition(double pos) {
    targetPos = pos;
    motor.setControl(control.withPosition(pos));
  }

  public Command setMotorPositionCmd(double pos) {
    return runOnce(() -> setMotorPosition(pos));
  }

  public Command setMotorPositionBetterCmd(WristPosition pos) {
    return runOnce(() -> setMotorPosition(pos.rotations));
  }

  public Command incrementMotorPositionForTesting(double inc) {
    return runOnce(() -> setMotorPosition(targetPos + inc));
  }

  public double getWristRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getWristTargetRotations() {
    return targetPos;
  }

  public double getWristDegrees() {
    return Degrees.convertFrom(getWristRotations(), Rotations);
  }

  public double getWristDegreesOffset() {
    return getWristDegrees() - ARM_OFFSET;
  }

  MechanismLigament2d wristMechanism;
  MechanismLigament2d wristRotatePart;
  MechanismLigament2d wristRotatePart2;
  MechanismLigament2d wristEnd;

  public MechanismLigament2d createMechanism2d() {
    wristMechanism =
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
    wristRotatePart2 =
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

  @Override
  public void periodic() {
    updateMechanism2d();
  }

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LEN, MASS),
          ARM_LEN,
          Radians.convertFrom(DOWN_LIMIT, Rotations),
          Radians.convertFrom(UP_LIMIT, Rotations),
          true,
          Radians.convertFrom(ARM_OFFSET, Degrees));

  @Override
  public void simulationPeriodic() {
    var motorSim = motor.getSimState();
    armSim.setInput(motorSim.getMotorVoltage());
    armSim.update(0.02);

    motorSim.setRotorVelocity(
        RotationsPerSecond.convertFrom(armSim.getVelocityRadPerSec(), RadiansPerSecond)
            * GEAR_RATIO);

    motorSim.setRawRotorPosition(
        Rotations.convertFrom(armSim.getAngleRads(), Radians) * GEAR_RATIO);
  }

  public VoltageOut sysIdControl = new VoltageOut(0);

  public SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("arm_sysid", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))), null, this));
}
