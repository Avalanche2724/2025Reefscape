package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator {
  // Constants
  private static final int ELEVATOR_ID = 41;
  private static final int ELEVATOR2_ID = 42;
  private static final double GEAR_RATIO = 14.0;
  private static final double MASS =
      Kilograms.convertFrom(30, Pounds); // estimate; only for simulation
  // extra factor of 2 because there are 2 sprockets pushing the elevator up? or something, idk:
  private static final double DRUM_RADIUS =
      Meters.convertFrom(0.25 / (2.0 * Math.sin(Math.toRadians(180.0 / 18.0))), Inches) * 2;
  public static final double MIN_HEIGHT = Meters.convertFrom(6.5, Inches); // 0.1651m
  public static final double MAX_HEIGHT = Meters.convertFrom(59.5, Inches); // 1.5113m
  private static final double CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS;
  private static final double METERS_PER_MOTOR_ROTATION = CIRCUMFERENCE / GEAR_RATIO;

  // I/O
  private final TalonFX motor = new TalonFX(ELEVATOR_ID);
  private final TalonFX followerMotor = new TalonFX(ELEVATOR2_ID);
  private final MotionMagicVoltage control = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent();

  private final VelocityTorqueCurrentFOC zeroingControl =
      new VelocityTorqueCurrentFOC(-0.5).withSlot(1).withIgnoreHardwareLimits(true);
  private final VelocityTorqueCurrentFOC algaeLaunchingControl = new VelocityTorqueCurrentFOC(1.5);

  public Elevator() {
    var config = new TalonFXConfiguration();

    config.Slot0.kP = 190.41;
    config.Slot0.kD = 14.148;
    config.Slot0.kS = 0.11697;
    config.Slot0.kV = 7.5487;
    config.Slot0.kA = 0.17841;
    config.Slot0.kG = 0.26462;

    // For zeroing sequence and algae launching
    config.Slot1.kP = 25;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    config.Feedback.SensorToMechanismRatio = 1 / METERS_PER_MOTOR_ROTATION;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HEIGHT + 0.01;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_HEIGHT;

    // TODO accel is slow right now because we need to implement wrist sequencing to avoid bumper
    config.MotionMagic.MotionMagicAcceleration = 0.4; // meters per second squared
    config.MotionMagic.MotionMagicCruiseVelocity = 1.4; // meters per second
    config.MotionMagic.MotionMagicJerk = 10; // meters per second cubed
    motor.getConfigurator().apply(config);

    motor.getClosedLoopError().setUpdateFrequency(50);
    motor.getClosedLoopReference().setUpdateFrequency(50);

    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerMotor.getConfigurator().apply(followerConfig);
    followerMotor.setControl(new Follower(ELEVATOR_ID, false));

    zero();
  }

  public void zero() {
    motor.setPosition(MIN_HEIGHT);
  }

  void setMotorPosition(double height) {
    motor.setControl(control.withPosition(height));
  }

  void setMotorDutyCycle(double d) {
    motor.set(d);
  }

  void setMotorAlgaeLaunchVelocity(double v) {
    motor.setControl(algaeLaunchingControl.withVelocity(v));
  }

  void setMotorZeroing() {
    motor.setControl(zeroingControl);
  }

  public double getElevatorHeight() {
    return motorPosition.getValueAsDouble();
  }

  private static final double STALL_DETECT_TORQUE = -9.5;
  private static final double VELOCITY_DETECT_THRESHOLD = 0.08;

  private boolean isStalling() {
    return motorTorqueCurrent.getValueAsDouble() < STALL_DETECT_TORQUE
        && Math.abs(motorVelocity.getValueAsDouble()) < VELOCITY_DETECT_THRESHOLD;
  }

  public Trigger isStalling = new Trigger(this::isStalling).debounce(0.1);

  public void periodic() {
    // TODO add telemetry
    motorPosition.refresh();
    motorVelocity.refresh();
    motorTorqueCurrent.refresh();
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
              Volts.of(0.5).div(Seconds.one()),
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("elevator_sysid", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
              null,
              Superstructure.instance));
}
