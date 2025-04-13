package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

@SuppressWarnings("FieldCanBeLocal") // Stop intellij complaints
public class Elevator {
  public static final double tempvar = 14.0 / (16.0 / 3.0);

  // Constants
  public static final int ELEVATOR_ID = 41;
  public static final int ELEVATOR2_ID = 42;
  // this is a bit wrong now but it's fine™
  public static final double MIN_HEIGHT = Meters.convertFrom(6.5, Inches);
  public static final double MAX_HEIGHT = 1.53;
  // Meters.convertFrom(59.06, Inches); // 1.5 m
  //
  private static final double GEAR_RATIO = 14.0 / tempvar;
  // Extra factor of 2 because there are 2 sprockets pushing up the elevato
  private static final double DRUM_RADIUS =
      Meters.convertFrom(0.25 / (2.0 * Math.sin(Math.toRadians(180.0 / 18.0))), Inches) * 2;
  private static final double CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS;
  // approx 1/23.212
  private static final double METERS_PER_MOTOR_ROTATION = CIRCUMFERENCE / GEAR_RATIO;
  // Other things
  private static final double STALL_DETECT_TORQUE = -9.5;
  private static final double VELOCITY_DETECT_THRESHOLD = 0.07;

  // Motors
  private final TalonFX motor = new TalonFX(ELEVATOR_ID);
  private final TalonFX followerMotor = new TalonFX(ELEVATOR2_ID);
  // Signals
  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent();
  Trigger isStallingTrigger = new Trigger(this::isStalling).debounce(0.25);
  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();
  private final StatusSignal<Voltage> motorSupplyVoltage = motor.getSupplyVoltage();
  // Control
  private final MotionMagicVoltage control = new MotionMagicVoltage(0);
  private final MotionMagicVelocityTorqueCurrentFOC smoothVelocityControl =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(2).withOverrideCoastDurNeutral(false);

  private final VelocityTorqueCurrentFOC unSmoothVelocityControl =
      new VelocityTorqueCurrentFOC(0).withSlot(1).withOverrideCoastDurNeutral(true);

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

    // From SysID (in theoretical analysis mode): max pos error 0.005, vel 0.05, control 7
    double kgHigh = 0.75;
    double kgLow = 0.38;

    config.Slot0.kP = Robot.isSimulation() ? 20 : 125;
    config.Slot0.kD = Robot.isSimulation() ? 0.1 : 1; // kD was having a skill issue?
    config.Slot0.kG = (kgHigh + kgLow) / 2;
    config.Slot0.kS = (kgHigh - kgLow) / 2;
    config.Slot0.kA = config.Slot0.kG / 9.8;
    config.Slot0.kV = 0.124 / 0.043080; // approx 2.88 V*s/m

    // Motion magic parameters
    config.MotionMagic.MotionMagicAcceleration = 6.5; // meters per second squared
    config.MotionMagic.MotionMagicCruiseVelocity =
        2.4; // meters per second; running somewhat lower than free speed
    config.MotionMagic.MotionMagicJerk = 35;

    // For zeroing sequence and going down
    config.Slot1.kG = 22.74; // estimate from voltage
    config.Slot1.kS = 7.44; // estimate from voltage
    config.Slot1.kP = 60;
    config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    // For algae launching
    config.Slot2.kG = config.Slot1.kG;
    config.Slot2.kS = config.Slot1.kS;
    config.Slot2.kP = 100;
    config.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    // Other things
    config.Feedback.SensorToMechanismRatio = 1 / METERS_PER_MOTOR_ROTATION;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Forward/reverse limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HEIGHT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // does this fix things?
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_HEIGHT;

    config.CurrentLimits.StatorCurrentLimit = 90;

    motor.getConfigurator().apply(config);

    // For debugging via hoot logs:
    // Make SysId a bit more accurate, hopefully
    motorPosition.setUpdateFrequency(200);
    motorVelocity.setUpdateFrequency(200);
    motorVoltage.setUpdateFrequency(200);
    motor.getClosedLoopError().setUpdateFrequency(50);
    motor.getClosedLoopReference().setUpdateFrequency(50);
    motor.getClosedLoopDerivativeOutput().setUpdateFrequency(50);
    motor.getClosedLoopOutput().setUpdateFrequency(50);
    motor.getClosedLoopProportionalOutput().setUpdateFrequency(50);
    motor.getClosedLoopFeedForward().setUpdateFrequency(50);

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
    motor.setControl(
        unSmoothVelocityControl.withVelocity(v).withIgnoreHardwareLimits(ignoreLimits));
  }

  private void setMotorVelocitySmooth(double v, boolean ignoreLimits) {
    motor.setControl(smoothVelocityControl.withVelocity(v).withIgnoreHardwareLimits(ignoreLimits));
  }

  void setMotorZeroingVelocity() {
    setMotorVelocity(-0.5, true);
  }

  void setMotorLaunchingVelocityUp() {
    setMotorVelocitySmooth(2.5, false);
  }

  // technically same as setmotorzeroingvelocity given lack of reverse limit
  void setMotorLaunchingVelocityDown() {
    motor.setControl(
        new TorqueCurrentFOC(5).withMaxAbsDutyCycle(0.3).withOverrideCoastDurNeutral(true));

    // setMotorVelocity(-0.5, false);
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

  double lastVel = 0;
  LinearFilter accelFilter = LinearFilter.movingAverage(8);

  public void simulationPeriodic(double dt) {
    var motorSim = motor.getSimState();
    m_elevatorSim.setInput(motorSim.getMotorVoltage());
    m_elevatorSim.update(dt);

    motorSim.setRawRotorPosition(
        (m_elevatorSim.getPositionMeters() - MIN_HEIGHT) / METERS_PER_MOTOR_ROTATION);
    double rotorVel = m_elevatorSim.getVelocityMetersPerSecond() / METERS_PER_MOTOR_ROTATION;
    motorSim.setRotorVelocity(
        m_elevatorSim.getVelocityMetersPerSecond() / METERS_PER_MOTOR_ROTATION);
    motorSim.setRotorAcceleration(accelFilter.calculate((lastVel - rotorVel) / dt));
    lastVel = rotorVel;
  }
}
