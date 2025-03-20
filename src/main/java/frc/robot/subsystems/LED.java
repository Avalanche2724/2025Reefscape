package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(
        runPattern(LEDPattern.solid(new Color(0x27, 0x24, 0xFF))).withName("blue default"));
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
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

  public Command netchecker() {
    var position = Robot.instance.robotContainer.drivetrain.getState().Pose.getX();
    if (position > 6.8 && position < 7.98) {
      return runPattern(LEDPattern.solid(Color.kGreen));
    } else {
      return runPattern(LEDPattern.solid(Color.kBlue));
    }
  }
}
