package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util;

/**
 * Elevator subsystem controlling vertical movement within the robot. Handles motion control,
 * position tracking, and simulation.
 */
public class Elevator {
  // -----------------------------------------------------------------------------
  // Physical Constants
  // -----------------------------------------------------------------------------
  private static final int ELEVATOR_PRIMARY_MOTOR_ID = 41;
  private static final int ELEVATOR_FOLLOWER_MOTOR_ID = 42;

  private static final double GEAR_RATIO = 14.0;
  private static final double MASS_KG = Kilograms.convertFrom(30, Pounds); // estimated mass
  private static final double DRUM_RADIUS_M =
      Meters.convertFrom(0.25 / (2.0 * Math.sin(Math.toRadians(180.0 / 18.0))), Inches) * 2;
  private static final double CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS_M;
  private static final double METERS_PER_MOTOR_ROTATION = CIRCUMFERENCE / GEAR_RATIO;

  // Mechanism position limits
  public static final double MIN_HEIGHT_M = Meters.convertFrom(6.5, Inches); // ~0.16m
  public static final double MAX_HEIGHT_M = Meters.convertFrom(55, Inches); // ~1.4m

  // Control constants
  private static final double STALL_CURRENT_THRESHOLD = -15;
  private static final double STALL_VELOCITY_THRESHOLD = 0.1;
  private static final double MOTION_MAGIC_ACCELERATION = 1.0; // m/s²
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 1.0; // m/s

  // -----------------------------------------------------------------------------
  // Hardware & Control Objects
  // -----------------------------------------------------------------------------
  private final TalonFX primaryMotor = new TalonFX(ELEVATOR_PRIMARY_MOTOR_ID);
  private final TalonFX followerMotor = new TalonFX(ELEVATOR_FOLLOWER_MOTOR_ID);
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> motorPosition = primaryMotor.getPosition();

  // SysID
  public final SysIdRoutine sysIdRoutine;

  // Visualization
  private MechanismLigament2d elevatorMechanism;

  // -----------------------------------------------------------------------------
  // Simulation
  // -----------------------------------------------------------------------------
  private final ElevatorSim elevatorSimulation =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          GEAR_RATIO,
          MASS_KG,
          DRUM_RADIUS_M,
          MIN_HEIGHT_M,
          MAX_HEIGHT_M,
          true,
          MIN_HEIGHT_M);

  /**
   * Creates a new Elevator subsystem. Configures motors and initializes the elevator at minimum
   * height.
   */
  public Elevator() {
    configureMotors();
    zero();

    // Initialize SysID routine using the shared utility
    this.sysIdRoutine =
        Util.createSysIdRoutine(
            primaryMotor,
            Superstructure.instance,
            "elevator",
            0.5, // 0.5 V/s ramp rate
            4.0 // 4.0 V maximum
            );
  }

  /** Configures the elevator motors with appropriate control parameters. */
  private void configureMotors() {
    var config = new TalonFXConfiguration();

    // PID and feedforward control parameters
    config.Slot0.kP = 244.15;
    config.Slot0.kD = 19.404;
    config.Slot0.kS = 0.11697;
    config.Slot0.kV = 7.5487;
    config.Slot0.kA = 0.17841;
    config.Slot0.kG = 0.26462;

    // Position feedback configuration
    config.Feedback.SensorToMechanismRatio = 1 / METERS_PER_MOTOR_ROTATION;

    // Motor output configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HEIGHT_M;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_HEIGHT_M;

    // Motion magic configuration
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

    // Apply configuration to primary motor
    primaryMotor.getConfigurator().apply(config);

    // Set up follower motor
    followerMotor.setControl(new Follower(ELEVATOR_PRIMARY_MOTOR_ID, false));
  }

  // -----------------------------------------------------------------------------
  // Public Interface
  // -----------------------------------------------------------------------------

  /** Resets the elevator position to the minimum height. */
  public void zero() {
    primaryMotor.setPosition(MIN_HEIGHT_M);
  }

  /**
   * Sets the target position for the elevator using motion magic control.
   *
   * @param height Target height in meters
   */
  public void setMotorPosition(double height) {
    primaryMotor.setControl(motionMagicControl.withPosition(height));
  }

  /**
   * Sets the duty cycle of the elevator motor directly.
   *
   * @param dutyCycle Duty cycle percentage (-1.0 to 1.0)
   */
  public void setMotorDutyCycle(double dutyCycle) {
    primaryMotor.set(dutyCycle);
  }

  /**
   * Gets the current elevator height.
   *
   * @return Current height in meters
   */
  public double getElevatorHeight() {
    return motorPosition.refresh().getValueAsDouble();
  }

  /**
   * Checks if the elevator is stalling (high current, low motion).
   *
   * @return True if stalling is detected
   */
  public boolean isStalling() {
    return primaryMotor.getTorqueCurrent().getValueAsDouble() < STALL_CURRENT_THRESHOLD
        && Math.abs(primaryMotor.getVelocity().getValueAsDouble()) < STALL_VELOCITY_THRESHOLD;
  }

  // -----------------------------------------------------------------------------
  // Visualization Methods
  // -----------------------------------------------------------------------------

  /**
   * Creates a mechanism visualization for the elevator.
   *
   * @return The mechanism ligament representing the elevator
   */
  public MechanismLigament2d createMechanism2d() {
    return elevatorMechanism =
        new MechanismLigament2d(
            "elevator",
            getElevatorHeight(),
            90,
            Centimeter.convertFrom(1, Inch),
            new Color8Bit(Color.kGreen));
  }

  /** Updates the elevator visualization with current height. */
  public void updateMechanism2d() {
    elevatorMechanism.setLength(getElevatorHeight());
  }

  // -----------------------------------------------------------------------------
  // Simulation Methods
  // -----------------------------------------------------------------------------

  /**
   * Updates the simulation model based on motor outputs.
   *
   * @param dt Time step in seconds
   */
  public void simulationPeriodic(double dt) {
    var motorSim = primaryMotor.getSimState();

    // Update the simulation with current voltage
    elevatorSimulation.setInput(motorSim.getMotorVoltage());
    elevatorSimulation.update(dt);

    // Feed simulation data back to motor
    motorSim.setRawRotorPosition(
        (elevatorSimulation.getPositionMeters() - MIN_HEIGHT_M) / METERS_PER_MOTOR_ROTATION);
    motorSim.setRotorVelocity(
        elevatorSimulation.getVelocityMetersPerSecond() / METERS_PER_MOTOR_ROTATION);
  }
}
