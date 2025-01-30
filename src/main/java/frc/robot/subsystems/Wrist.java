package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private static final int WRIST_ID = 30;
  public static final double GEAR_RATIO = 30.0;
  public static final double MASS = 4.0;
  public static final double ARM_LEN = Meters.convertFrom(10, Inches);
  public static final double UP_LIMIT = Rotations.convertFrom(90, Degrees);
  public static final double DOWN_LIMIT = Rotations.convertFrom(-90, Degrees);
  public static final double ARM_START = 0;

  public enum WristPosition {
    BASE(0),
    SCORE(-30),
    UP(30);

    public final double rotation;

    WristPosition(double degrees) {
      this.rotation = Rotations.convertFrom(degrees, Degrees);
    }
  }

  private final PositionVoltage control = new PositionVoltage(0);

  private final TalonFX motor = new TalonFX(WRIST_ID);

  public Wrist() {
    var config = new TalonFXConfiguration();
    config.Slot0.kP = 5;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    motor.getConfigurator().apply(config);
    motor.setPosition(0);
  }

  private void setMotorPosition(double pos) {
    motor.setControl(control.withPosition(pos));
  }

  public Command setMotorPositionCmd(double pos) {
    return runOnce(() -> setMotorPosition(pos));
  }

  public double getWristRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getWristDegrees() {
    return Degrees.convertFrom(getWristRotations(), Rotations);
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
