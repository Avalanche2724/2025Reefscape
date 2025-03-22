package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@SuppressWarnings("FieldCanBeLocal") // Stop intellij complaints
public class Elevator {
  // Constants
  public static final int ELEVATOR_ID = 41;
  public static final int ELEVATOR2_ID = 42;
  // this is a bit wrong now but it's fine™
  public static final double MIN_HEIGHT = Meters.convertFrom(6.5, Inches);
  public static final double MAX_HEIGHT = 1.56;
  // Meters.convertFrom(59.06, Inches); // 1.5 m
  //
  private static final double GEAR_RATIO = 14.0;
  // Extra factor of 2 because there are 2 sprockets pushing up the elevato
  private static final double DRUM_RADIUS =
      Meters.convertFrom(0.25 / (2.0 * Math.sin(Math.toRadians(180.0 / 18.0))), Inches) * 2;
  private static final double CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS;
  private static final double METERS_PER_MOTOR_ROTATION = CIRCUMFERENCE / GEAR_RATIO;
  // Other things
  private static final double STALL_DETECT_TORQUE = -9.5;
  private static final double VELOCITY_DETECT_THRESHOLD = 0.08;
  private static final double ELEVATOR_ZEROING_VELOCITY = -0.5;
  private static final double ALGAE_LAUNCHING_VELOCITY = 1.5;

  // Motors
  private final TalonFX motor = new TalonFX(ELEVATOR_ID);
  private final TalonFX followerMotor = new TalonFX(ELEVATOR2_ID);
  // Signals
  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent();
  Trigger isStallingTrigger = new Trigger(this::isStalling).debounce(0.25);
  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
  // Control
  private final MotionMagicVoltage control = new MotionMagicVoltage(0);
  private final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(0).withSlot(1);
  // Simulation
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          GEAR_RATIO,
          Kilograms.convertFrom(30, Pounds), // estimate
          DRUM_RADIUS,
          MIN_HEIGHT,
          MAX_HEIGHT,
          true,
          MIN_HEIGHT);
  // Mechanism2d
  public MechanismLigament2d elevatorMechanism;
  // SysId
  public VoltageOut sysIdControl = new VoltageOut(0);

  @SuppressWarnings("unused")
  public SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("Elevator_SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
              null,
              Superstructure.instance));

  public Elevator() {
    var config = new TalonFXConfiguration();

    // Slot for regular positional control
    // From SysID: max pos error 0.005, vel 0.05, control 12
    config.Slot0.kP = 198.96;
    config.Slot0.kD = 13.403;
    config.Slot0.kS = 0.1063;
    config.Slot0.kV = 7.6932;
    config.Slot0.kA = 0.10934;
    config.Slot0.kG = 0.2857;

    // For zeroing sequence and algae launching
    config.Slot1.kP = 15;
    config.Slot1.kS = 4.5; // Estimated from voltage kS
    config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 20;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -20;
    // Motion magic parameters
    config.MotionMagic.MotionMagicAcceleration = 3.2; // meters per second squared
    config.MotionMagic.MotionMagicCruiseVelocity = 1.4; // meters per second
    // config.MotionMagic.MotionMagicJerk = 20; // meters per second cubed

    // Other things
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.Feedback.SensorToMechanismRatio = 1 / METERS_PER_MOTOR_ROTATION;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Forward/reverse limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HEIGHT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // does this fix things?
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_HEIGHT;

    motor.getConfigurator().apply(config);

    // For debugging via hoot logs:
    motor.getClosedLoopError().setUpdateFrequency(50);
    motor.getClosedLoopReference().setUpdateFrequency(50);
    // Make SysId a bit more accurate, hopefully
    motorPosition.setUpdateFrequency(200);
    motorVelocity.setUpdateFrequency(200);
    motorVoltage.setUpdateFrequency(200);

    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.CurrentLimits.StatorCurrentLimit = config.CurrentLimits.StatorCurrentLimit;
    followerMotor.getConfigurator().apply(followerConfig);
    followerMotor.setControl(new Follower(ELEVATOR_ID, false));

    zeroElevatorPosition();
  }

  public void periodic() {
    motorPosition.refresh();
    motorVelocity.refresh();
    motorTorqueCurrent.refresh();
    motorVoltage.refresh();
    // Log to NetworkTables
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
    SmartDashboard.putNumber("Elevator Current", getTorqueCurrent());
    SmartDashboard.putNumber("Elevator Voltage", getVoltage());
  }

  // Reading signals
  public double getElevatorHeight() {
    return motorPosition.getValueAsDouble();
  }

  boolean isStalling() {
    return getTorqueCurrent() < STALL_DETECT_TORQUE
        && Math.abs(getVelocity()) < VELOCITY_DETECT_THRESHOLD;
  }

  public double getVelocity() {
    return motorVelocity.getValueAsDouble();
  }

  public double getTorqueCurrent() {
    return motorTorqueCurrent.getValueAsDouble();
  }

  public double getVoltage() {
    return motorVoltage.getValueAsDouble();
  }

  // Controls

  void zeroElevatorPosition() {
    motor.setPosition(MIN_HEIGHT, 0);
  }

  void setMotorPosition(double height) {
    motor.setControl(control.withPosition(height));
  }

  void stopMotor() {
    motor.set(0);
  }

  private void setMotorVelocity(double v, boolean ignoreLimits) {
    motor.setControl(velocityControl.withVelocity(v).withIgnoreHardwareLimits(ignoreLimits));
  }

  void setMotorZeroingVelocity() {
    setMotorVelocity(ELEVATOR_ZEROING_VELOCITY, true);
  }

  void setMotorLaunchingVelocityUp() {
    motor.set(1);
    // setMotorVelocity(ALGAE_LAUNCHING_VELOCITY, false);
  }

  void setMotorLaunchingVelocityDown() {
    motor.set(-1);
    // setMotorVelocity(-ALGAE_LAUNCHING_VELOCITY, false);
  }

  // Mechanism visualization and simulation stuff

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

  public void simulationPeriodic(double dt) {
    var motorSim = motor.getSimState();
    m_elevatorSim.setInput(motorSim.getMotorVoltage());
    m_elevatorSim.update(dt);

    motorSim.setRawRotorPosition(
        (m_elevatorSim.getPositionMeters() - MIN_HEIGHT) / METERS_PER_MOTOR_ROTATION);
    motorSim.setRotorVelocity(
        m_elevatorSim.getVelocityMetersPerSecond() / METERS_PER_MOTOR_ROTATION);
  }
}
