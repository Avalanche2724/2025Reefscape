package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Wrist;
import java.util.List;
import java.util.stream.Stream;

public class Util {
  public static void configureUserButton() {
    var canMotors =
        Stream.concat(
                Stream.of(
                        TunerConstants.FrontLeft.DriveMotorId,
                        TunerConstants.FrontRight.DriveMotorId,
                        TunerConstants.BackLeft.DriveMotorId,
                        TunerConstants.BackRight.DriveMotorId,
                        TunerConstants.FrontLeft.SteerMotorId,
                        TunerConstants.FrontRight.SteerMotorId,
                        TunerConstants.BackLeft.SteerMotorId,
                        TunerConstants.BackRight.SteerMotorId)
                    .map(id -> new DeviceIdentifier(id, "talon fx", "CANivore")),
                Stream.of(
                        Elevator.ELEVATOR_ID,
                        Elevator.ELEVATOR2_ID,
                        Wrist.WRIST_ID,
                        Intake.LEFTMOTOR_ID,
                        Intake.RIGHTMOTOR_ID)
                    .map(id -> new DeviceIdentifier(id, "talon fx", "")))
            .map(TalonFXConfigurator::new)
            .toList();

    var userButton =
        new Trigger(RobotController::getUserButton)
            .debounce(0.1, Debouncer.DebounceType.kRising)
            .debounce(5, Debouncer.DebounceType.kFalling)
            .and(DriverStation::isDisabled);

    userButton.whileTrue(
        Commands.startEnd(
                () -> setMotorBrake(canMotors, false), () -> setMotorBrake(canMotors, true))
            .withTimeout(5) // todo: necessary?
            .ignoringDisable(true));
  }

  private static void setMotorBrake(List<TalonFXConfigurator> configurator, boolean brake) {
    for (var conf : configurator) {
      var motorOutputConfigs = new MotorOutputConfigs();
      var retval = conf.refresh(motorOutputConfigs);
      if (retval.isOK()) {
        motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        conf.apply(motorOutputConfigs, 0);
      }
    }
  }
}
