package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
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

    // Set the default command to netchecker, which now includes our alternating white-blue
    // animation.
    setDefaultCommand(netchecker().withName("ledchanger"));
  }

  @Override
  public void periodic() {
    // Continuously update the LED strip with the current buffer data.
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
            if (!coralMode) {
              LEDPattern.solid(Color.kBlue).applyTo(m_buffer);
            } else {
              // Create a base alternating pattern: even indices are white, odd indices are blue.
              LEDPattern alternating =
                  (reader, writer) -> {
                    int len = reader.getLength();
                    for (int i = 0; i < len; i++) {
                      if (i % 3 == 0) {
                        writer.setLED(i, Color.kWhite);
                      } else {
                        writer.setLED(i, Color.kBlue);
                      }
                    }
                  };
              // Animate the pattern by scrolling it slowly. Adjust the speed here as desired.
              alternating.scrollAtRelativeSpeed(Percent.per(Second).of(-40)).applyTo(m_buffer);
            }
          }
          // Update the LED strip with the new data.
          m_led.setData(m_buffer);
        })
        .ignoringDisable(true);
  }
}
