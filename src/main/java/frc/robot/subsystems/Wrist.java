package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private static final int WRIST_ID = 30;
  public static final double GEAR_RATIO = 30.0;
  public static final double MASS = 4.0;
  public static final double ARM_LEN = Meters.convertFrom(24, Inches);
  // angle from arm flat to center of gravity; approximated rn
  public static final double ARM_OFFSET = -9.0;
  public static final double UP_LIMIT = Rotations.convertFrom(90 + ARM_OFFSET, Degrees);
  public static final double DOWN_LIMIT = Rotations.convertFrom(-90 + ARM_OFFSET, Degrees);
  public static final double ARM_START = 0;

  public enum WristPosition {
    BASE(0),
    SCORE(-30),
    UP(30);

    public final double rotation;

    WristPosition(double degrees) {
      this.rotation = Rotations.convertFrom(degrees + ARM_OFFSET, Degrees);
    }
  }

  private final PositionVoltage control = new PositionVoltage(0);

  private final TalonFX motor = new TalonFX(WRIST_ID);

  public Wrist() {
    var config = new TalonFXConfiguration();
    config.Slot0.kP = 6;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    motor.getConfigurator().apply(config);
    motor.setPosition(0);
  }

  double targetPos;

  private void setMotorPosition(double pos) {
    targetPos = pos;
    motor.setControl(control.withPosition(pos));
  }

  public Command setMotorPositionCmd(double pos) {
    return runOnce(() -> setMotorPosition(pos));
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
          Radians.convertFrom(ARM_START, Rotations));

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
}
