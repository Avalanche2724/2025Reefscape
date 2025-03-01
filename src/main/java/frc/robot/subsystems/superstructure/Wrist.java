package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util;

/**
 * Wrist subsystem controlling angular movement of the robot's end effector. Handles angular
 * position control, simulation, and visualization.
 */
public class Wrist {
  // -----------------------------------------------------------------------------
  // Physical Constants
  // -----------------------------------------------------------------------------
  private static final int MOTOR_ID = 51;
  public static final double GEAR_RATIO = 64.0;
  public static final double MASS_KG = Kilograms.convertFrom(15, Pounds);
  public static final double ARM_LENGTH_M = Meters.convertFrom(30, Inches);

  // Angular offsets and limits
  public static final double ARM_OFFSET_DEG = Degrees.convertFrom(-0.072131, Rotations);
  public static final double ARM_OFFSET_ROT = Rotations.convertFrom(ARM_OFFSET_DEG, Degrees);
  public static final double UPPER_LIMIT_ROT = Rotations.convertFrom(90 + ARM_OFFSET_DEG, Degrees);
  public static final double LOWER_LIMIT_ROT = Rotations.convertFrom(-90 + ARM_OFFSET_DEG, Degrees);

  // Control constants
  private static final double MOTION_MAGIC_ACCELERATION = 0.5; // rotations per second squared
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 1.0; // rotations per second

  // -----------------------------------------------------------------------------
  // Hardware & Control Objects
  // -----------------------------------------------------------------------------
  private final TalonFX motor = new TalonFX(MOTOR_ID);
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

  // SysID Control
  public final SysIdRoutine sysIdRoutine;

  // -----------------------------------------------------------------------------
  // Visualization
  // -----------------------------------------------------------------------------
  private MechanismLigament2d wristRotatePart;

  // -----------------------------------------------------------------------------
  // Simulation
  // -----------------------------------------------------------------------------
  private final SingleJointedArmSim armSimulation =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, MASS_KG),
          ARM_LENGTH_M,
          Radians.convertFrom(LOWER_LIMIT_ROT, Rotations),
          Radians.convertFrom(UPPER_LIMIT_ROT, Rotations),
          true,
          Radians.convertFrom(ARM_OFFSET_ROT, Rotations));

  /** Creates a new Wrist subsystem. Configures motors and initializes the wrist position. */
  public Wrist() {
    configureMotor();
    zero();

    // Initialize SysID routine using the shared utility
    this.sysIdRoutine =
        Util.createSysIdRoutine(
            motor,
            Superstructure.instance,
            "arm",
            0.5, // 0.5 V/s ramp rate (using default)
            3.0 // 3.0 V step voltage
            );
  }

  /** Configures the wrist motor with appropriate control parameters. */
  private void configureMotor() {
    var config = new TalonFXConfiguration();

    // PID and feedforward control parameters
    config.Slot0.kP = 70;
    config.Slot0.kD = 2.5;
    config.Slot0.kS = 0.038097;
    config.Slot0.kV = 7.9144;
    config.Slot0.kA = 0.10046;
    config.Slot0.kG = 0.55565;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion magic configuration
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

    // Position feedback configuration
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = UPPER_LIMIT_ROT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = LOWER_LIMIT_ROT;

    // Motor output configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply configuration
    motor.getConfigurator().apply(config);
  }

  // -----------------------------------------------------------------------------
  // Public Interface
  // -----------------------------------------------------------------------------

  /** Resets the wrist position to the arm offset position. */
  public void zero() {
    motor.setPosition(ARM_OFFSET_ROT);
  }

  /**
   * Sets the target position for the wrist using motion magic control.
   *
   * @param position Target position in rotations
   */
  private void setMotorRotations(double position) {
    motor.setControl(motionMagicControl.withPosition(position));
  }

  /**
   * Sets the wrist angle in degrees relative to the arm offset.
   *
   * @param degrees Target angle in degrees (offset-compensated)
   */
  public void setMotorDegreesOffset(double degrees) {
    setMotorRotations(Rotations.convertFrom(degrees + ARM_OFFSET_DEG, Degrees));
  }

  /**
   * Sets the duty cycle of the wrist motor directly.
   *
   * @param dutyCycle Duty cycle percentage (-1.0 to 1.0)
   */
  public void setMotorDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  /**
   * Gets the current wrist position in rotations.
   *
   * @return Current position in rotations
   */
  public double getWristRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  /**
   * Gets the current wrist angle in degrees.
   *
   * @return Current angle in degrees
   */
  public double getWristDegrees() {
    return Degrees.convertFrom(getWristRotations(), Rotations);
  }

  /**
   * Gets the current wrist angle in degrees relative to the arm offset.
   *
   * @return Current offset-compensated angle in degrees
   */
  public double getWristDegreesOffset() {
    return getWristDegrees() - ARM_OFFSET_DEG;
  }

  // -----------------------------------------------------------------------------
  // Visualization Methods
  // -----------------------------------------------------------------------------

  /**
   * Creates a mechanism visualization for the wrist.
   *
   * @return The mechanism ligament representing the wrist base
   */
  public MechanismLigament2d createMechanism2d() {
    var wristMechanism =
        new MechanismLigament2d(
            "wrist_base_elev_part_1",
            Meters.convertFrom(2, Inch),
            -90,
            0,
            new Color8Bit(Color.kRed));

    wristRotatePart =
        wristMechanism.append(
            new MechanismLigament2d(
                "wrist_rotate_part_2",
                Meters.convertFrom(11, Inches),
                getWristDegrees(),
                Centimeters.convertFrom(1, Inch),
                new Color8Bit(Color.kPurple)));

    var wristRotatePart2 =
        wristRotatePart.append(
            new MechanismLigament2d(
                "wrist_rotate_part_3",
                Meters.convertFrom(4, Inches),
                90,
                Centimeters.convertFrom(1, Inch),
                new Color8Bit(Color.kPurple)));

    // Create and return wristEnd here but don't store the reference
    wristRotatePart2.append(
        new MechanismLigament2d(
            "wrist_end_part_4",
            Meters.convertFrom(13, Inches),
            -90,
            Centimeters.convertFrom(4, Inches),
            new Color8Bit(Color.kBlue)));

    return wristMechanism;
  }

  /** Updates the wrist visualization with the current angle. */
  public void updateMechanism2d() {
    wristRotatePart.setAngle(getWristDegreesOffset());
  }

  // -----------------------------------------------------------------------------
  // Simulation Methods
  // -----------------------------------------------------------------------------

  /**
   * Updates the simulation model based on motor outputs.
   *
   * @param deltaTime Time step in seconds
   */
  public void simulationPeriodic(double deltaTime) {
    var motorSim = motor.getSimState();

    // Update the simulation with current voltage
    armSimulation.setInput(motorSim.getMotorVoltage());
    armSimulation.update(deltaTime);

    // Feed simulation data back to motor
    motorSim.setRotorVelocity(
        RotationsPerSecond.convertFrom(armSimulation.getVelocityRadPerSec(), RadiansPerSecond)
            * GEAR_RATIO);
    motorSim.setRawRotorPosition(
        Rotations.convertFrom(armSimulation.getAngleRads(), Radians) * GEAR_RATIO);
  }
}
