package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
  private static final int ELEVATOR_ID = 40;
  private static final int ELEVATOR2_ID = 41;

  public static final double GEAR_RATIO = 20.0;
  public static final double MASS = Kilograms.convertFrom(25, Pounds); // estimate
  public static final double DRUM_RADIUS = Meters.convertFrom(0.95766, Inches);
  public static final double MIN_HEIGHT = Meters.convertFrom(3.75, Inches);
  public static final double MAX_HEIGHT = Meters.convertFrom(3.75 + 59.5, Inches);
  public static final double CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS;

  public double heightToRotations(double height) {
    return height / CIRCUMFERENCE;
  }

  public double rotationsToHeight(double rotations) {
    return rotations * CIRCUMFERENCE;
  }

  public enum ElevatorPosition {
    BASE(MIN_HEIGHT),
    L1_SCORE(0.5),
    L2_ALGAE(0.8),
    L2_SCORE(1.1),
    L3_ALGAE(1.3),
    L3_SCORE(1.5),
    L4_SCORE(1.7),
    TOP(2.0);

    public final double distance;

    ElevatorPosition(double distance) {
      this.distance = distance;
    }
  }

  private final PositionVoltage control = new PositionVoltage(0);

  private final TalonFX motor = new TalonFX(ELEVATOR_ID);
  private final TalonFX followerMotor = new TalonFX(ELEVATOR_ID);

  private final StatusSignal<Angle> motorPosition = motor.getPosition();

  public Elevator() {
    var config = new TalonFXConfiguration();
    config.Slot0.kP = 12;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    followerMotor.setControl(new Follower(ELEVATOR2_ID, true));
    motor.getConfigurator().apply(config);
  }

  private void setMotorPosition(double height) {
    motor.setControl(control.withPosition(heightToRotations(height)));
  }

  private void setMotorPosition(ElevatorPosition pos) {
    setMotorPosition(pos.distance);
  }

  public Command setMotorPositionCmd(ElevatorPosition pos) {
    return runOnce(() -> setMotorPosition(pos));
  }

  public Command incrementMotorPositionForTesting(double inc) {
    return runOnce(() -> setMotorPosition(getElevatorHeight() + inc));
  }

  public double getElevatorRotations() {
    return motorPosition.refresh().getValueAsDouble();
  }

  public double getElevatorHeight() {
    return rotationsToHeight(getElevatorRotations());
  }

  public MechanismLigament2d elevatorMechanism;

  public MechanismLigament2d createMechanism2d() {
    return elevatorMechanism =
        new MechanismLigament2d(
            "elevator",
            getElevatorHeight(),
            90,
            Centimeter.convertFrom(1, Inch),
            new Color8Bit(Color.kGreen));
  }

  public void updateMechanism2d() {
    elevatorMechanism.setLength(getElevatorHeight());
  }

  @Override
  public void periodic() {
    updateMechanism2d();
  }

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          GEAR_RATIO,
          MASS,
          DRUM_RADIUS,
          MIN_HEIGHT,
          MAX_HEIGHT,
          true,
          MIN_HEIGHT);

  @Override
  public void simulationPeriodic() {
    var motorSim = motor.getSimState();
    m_elevatorSim.setInput(motorSim.getMotorVoltage());
    m_elevatorSim.update(0.02);

    motorSim.setRawRotorPosition(GEAR_RATIO * heightToRotations(m_elevatorSim.getPositionMeters()));
    motorSim.setRotorVelocity(
        GEAR_RATIO * heightToRotations(m_elevatorSim.getVelocityMetersPerSecond()));
  }

  public VoltageOut sysIdControl = new VoltageOut(0);

  public SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(6),
              null,
              (state) -> SignalLogger.writeString("elevator_sysid", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))), null, this));
}
