package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** Utility class containing shared functionality across the robot. */
public class Util {

  /**
   * Creates a SysIdRoutine for a TalonFX motor with a shared configuration pattern.
   *
   * @param motor The TalonFX motor to create a SysIdRoutine for
   * @param subsystem The subsystem that this motor belongs to
   * @param name A name identifier for this routine in logs (e.g., "elevator", "arm")
   * @param voltageRamp Voltage ramp rate in volts per second (null for default)
   * @param stepVoltage Step voltage to apply during test (null for default)
   * @return A SysIdRoutine configured for the motor
   */
  public static SysIdRoutine createSysIdRoutine(
      TalonFX motor, Subsystem subsystem, String name, double voltageRamp, double stepVoltage) {

    // Create a local VoltageOut control to be captured by the lambda
    VoltageOut sysIdControl = new VoltageOut(0);

    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            Volts.of(voltageRamp).div(Seconds.one()),
            Volts.of(stepVoltage),
            null,
            (state) -> SignalLogger.writeString(name + "_sysid", state.toString()));

    return new SysIdRoutine(
        config,
        new SysIdRoutine.Mechanism(
            (volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
            null,
            subsystem));
  }
}
