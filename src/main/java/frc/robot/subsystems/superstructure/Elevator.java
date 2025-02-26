package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator {
  // Constants
  private static final int ELEVATOR_ID = 41;
  private static final int ELEVATOR2_ID = 42;
  private static final double GEAR_RATIO = 14.0;
  private static final double MASS = Kilograms.convertFrom(30, Pounds); // estimate
  private static final double DRUM_RADIUS =
      Meters.convertFrom(0.25 / (2.0 * Math.sin(Math.toRadians(180.0 / 18.0))), Inches);
  public static final double MIN_HEIGHT = Meters.convertFrom(6.5, Inches); // 0.16m ish
  public static final double MAX_HEIGHT = Meters.convertFrom(55, Inches); // 1.4m ish
  private static final double CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS;
  private static final double METERS_PER_MOTOR_ROTATION = CIRCUMFERENCE / GEAR_RATIO;

  // I/O
  private final TalonFX motor = new TalonFX(ELEVATOR_ID);
  private final TalonFX followerMotor = new TalonFX(ELEVATOR2_ID);
  private final MotionMagicVoltage control = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> motorPosition = motor.getPosition();

  public Elevator() {
    var config = new TalonFXConfiguration();

    config.Slot0.kP = 603.43;
    config.Slot0.kD = 22.735;
    config.Slot0.kS = 0.045578;
    config.Slot0.kV = 15.212;
    config.Slot0.kA = 0.27364;
    config.Slot0.kG = 0.27351;

    config.Feedback.SensorToMechanismRatio = 1 / METERS_PER_MOTOR_ROTATION;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HEIGHT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_HEIGHT;

    config.MotionMagic.MotionMagicAcceleration = 2; // meters per second squared
    config.MotionMagic.MotionMagicCruiseVelocity = 0.8; // meters per second
    motor.getConfigurator().apply(config);

    followerMotor.setControl(new Follower(ELEVATOR_ID, false));
    motor.setPosition(MIN_HEIGHT);
  }

  void setMotorPosition(double height) {
    motor.setControl(control.withPosition(height));
  }

  void setMotorDutyCycle(double d) {
    motor.set(d);
  }

  public double getElevatorHeight() {
    return motorPosition.refresh().getValueAsDouble();
  }

  // Mechanism2d
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

  // Simulation
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

  public void simulationPeriodic(double dt) {
    var motorSim = motor.getSimState();
    m_elevatorSim.setInput(motorSim.getMotorVoltage());
    m_elevatorSim.update(dt);

    motorSim.setRawRotorPosition(
        (m_elevatorSim.getPositionMeters() - MIN_HEIGHT) / METERS_PER_MOTOR_ROTATION);
    motorSim.setRotorVelocity(
        m_elevatorSim.getVelocityMetersPerSecond() / METERS_PER_MOTOR_ROTATION);
  }

  // SysId
  public VoltageOut sysIdControl = new VoltageOut(0);

  public SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("elevator_sysid", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
              null,
              Superstructure.instance));
}
