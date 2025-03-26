package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.AllianceFlipUtil;

public class LED extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 18;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public LED() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to continuously run netchecker,
    // which now includes the white-blue animation instead of solid orange.
    setDefaultCommand(netchecker().withName("ledchanger"));
  }

  @Override
  public void periodic() {
    // Update the LED strip with the current buffer data.
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a given LED pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   * @return a command that applies the pattern
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer)).ignoringDisable(true);
  }

  public Command thingy() {
    return runPattern(
        LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(90)));
  }

  public Command thingy2() {
    return runPattern(LEDPattern.solid(Color.kBlack));
  }

  /**
   * This command checks various robot states and applies different LED patterns. If none of the
   * special conditions are met, it displays a slow white-blue animation.
   */
  public Command netchecker() {
    return run(() -> {
          var position = Robot.instance.robotContainer.drivetrain.getState().Pose.getX();
          var hasGamePiece = Robot.instance.robotContainer.intake.hasGamePiece();
          var coralMode = Robot.instance.robotContainer.controls.isOnCoralBindings;

          if (Math.abs(position - AllianceFlipUtil.applyX(7.4)) < Meters.convertFrom(1.5, Inch)) {
            LEDPattern.solid(Color.kRed).applyTo(m_buffer);
            Robot.instance.robotContainer.controls.driver.rumble.accept(0.7);
          } else if (hasGamePiece) {
            LEDPattern.solid(Color.kGreen).applyTo(m_buffer);
            Robot.instance.robotContainer.controls.driver.rumble.accept(0.2);
          } else {
            Robot.instance.robotContainer.controls.driver.rumble.accept(0);
            if (coralMode) {
              LEDPattern.solid(Color.kBlue).applyTo(m_buffer);
            } else {
              // White-Blue Animation: smoothly interpolate between blue (0,0,255) and white
              // (255,255,255)
              double periodMicros = 10_000_000.0; // 10-second cycle
              long now = RobotController.getTime();
              double t = (now % (long) periodMicros) / periodMicros;
              double phase = t * 2 * Math.PI;
              double lerp = (Math.sin(phase) + 1) / 2.0;

              int red = (int) (0 + (255 - 0) * lerp);
              int green = (int) (0 + (255 - 0) * lerp);
              int blue = 255;

              for (int i = 0; i < m_buffer.getLength(); i++) {
                m_buffer.setLED(i, new Color(red, green, blue));
              }
            }
          }
          // Update the LED strip with the new data.
          m_led.setData(m_buffer);
        })
        .ignoringDisable(true);
  }
}
