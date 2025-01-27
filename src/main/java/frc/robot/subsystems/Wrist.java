package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private static final int WRIST_ID = 30;

  private static final double GEAR_RATIO = 20.0;
  private static final Mass MASS = Kilograms.of(4.0);
  private static final Distance ARM_LEN = Inches.of(10);
  private static final Angle UP_LIMIT = Degrees.of(90);
  private static final Angle DOWN_LIMIT = Degrees.of(-90);
  private static final Angle ARM_START = Degrees.of(0);

  public enum WristPosition {
    BASE(Degrees.of(0)),
    SCORE(Degrees.of(-30)),
    UP(Degrees.of(30));

    public final Angle rotation;

    WristPosition(Angle rotation) {
      this.rotation = rotation;
    }
  }

  private final PositionVoltage control = new PositionVoltage(0);

  private final TalonFX motor = new TalonFX(WRIST_ID);

  public Wrist() {
    var config = new TalonFXConfiguration();
    config.Slot0.kP = 1;
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motor.getConfigurator().apply(config);
    motor.setPosition(0);
  }

  private void setMotorPosition(double pos) {
    motor.setControl(control.withPosition(pos));
  }

  public Command setMotorPositionCmd(double pos) {
    return runOnce(() -> setMotorPosition(pos));
  }

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LEN.in(Meters), MASS.in(Kilograms)),
          ARM_LEN.in(Meters),
          DOWN_LIMIT.in(Radians),
          UP_LIMIT.in(Radians),
          true,
          ARM_START.in(Radians));

  @Override
  public void simulationPeriodic() {
    var motorSim = motor.getSimState();
    armSim.setInput(motorSim.getMotorVoltage());
    armSim.update(0.02);

    motorSim.setRotorVelocity(
        RadiansPerSecond.of(armSim.getVelocityRadPerSec())
            .times(GEAR_RATIO)
            .in(RotationsPerSecond));

    motorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads()).times(GEAR_RATIO).in(Rotations));
  }
}
