package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private static final int WRIST_ID = 30;

  private static final double GEAR_RATIO = 20.0;
  private static final Mass MASS = Kilograms.of(4.0);
  private static final Distance DRUM_RADIUS = Inches.of(2.0);
  private static final Distance MIN_HEIGHT = Meters.of(0.0);
  private static final Distance MAX_HEIGHT = Meters.of(5.0);

  private final PositionVoltage control = new PositionVoltage(0);

  /*private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
        GEAR_RATIO,
  SingleJointedArmSim.estimateMOI(10.0, 10.0),
  double armLengthMeters,
  double minAngleRads,
  double maxAngleRads,
  boolean simulateGravity,
  Rotation.of(0).in(Radians),
  )*/

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

  @Override
  public void simulationPeriodic() {
    /*var motorSim = motor.getSimState();
    m_arm.setInput(motorSim.getMotorVoltage());
    m_elevatorSim.update(0.02);

    motorSim.setRotorVelocity(
        Meters.of(m_elevatorSim.getVelocityMetersPerSecond())
            .div(DRUM_RADIUS.times(2 * Math.PI))
            .times(Rotation.one())
            .div(Seconds.one())
            .times(GEAR_RATIO));

    motorSim.setRawRotorPosition(
        Meters.of(m_elevatorSim.getPositionMeters())
            .div(DRUM_RADIUS.times(2 * Math.PI))
            .times(Rotation.one())
            .times(GEAR_RATIO));

    BatterySim.calculateDefaultBatteryLoadedVoltage(null);*/
  }
}
