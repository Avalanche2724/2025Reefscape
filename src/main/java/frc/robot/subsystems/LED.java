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

    // Set the default command to netchecker which now includes blue fire animation
    setDefaultCommand(netchecker().withName("ledchanger"));
  }

  @Override
  public void periodic() {
    // Continuously update the LED strip with the current buffer data
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a given LED pattern on the entire LED strip
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

  /** Command that creates a blue fire animation with flickering effects */
  public Command blueFire() {
    return run(() -> {
          // Blue fire animation
          for (int i = 0; i < kLength; i++) {
            int baseBlue = 255;
            int red = (int) (Math.random() * 60);
            int green = (int) (Math.random() * 100);
            int blue = baseBlue - (int) (Math.random() * 80);

            if (i < kLength / 3) {
              blue = Math.min(255, blue + 50);
              green = Math.min(255, green + 30);
            }

            if (Math.random() > 0.7) {
              blue = Math.min(255, blue + (int) (Math.random() * 50));
            }

            m_buffer.setLED(i, new Color(red, green, blue));
          }

          if (Math.random() > 0.9) {
            int sparkPos = (int) (Math.random() * kLength);
            m_buffer.setLED(sparkPos, Color.kWhite);
          }

          m_led.setData(m_buffer);
        })
        .ignoringDisable(true);
  }

  /**
   * The netchecker command evaluates robot state and applies different LED patterns: - If near
   * target position: solid red - If holding game piece: solid green - Otherwise if coral mode off:
   * solid blue - If coral mode on: blue fire animation
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
            if (!coralMode) {
              LEDPattern.solid(Color.kBlue).applyTo(m_buffer);
            } else {
              // Blue fire animation
              for (int i = 0; i < kLength; i++) {
                int baseBlue = 255;
                int red = (int) (Math.random() * 60);
                int green = (int) (Math.random() * 100);
                int blue = baseBlue - (int) (Math.random() * 80);

                if (i < kLength / 3) {
                  blue = Math.min(255, blue + 50);
                  green = Math.min(255, green + 30);
                }

                if (Math.random() > 0.7) {
                  blue = Math.min(255, blue + (int) (Math.random() * 50));
                }

                m_buffer.setLED(i, new Color(red, green, blue));
              }

              if (Math.random() > 0.9) {
                int sparkPos = (int) (Math.random() * kLength);
                m_buffer.setLED(sparkPos, Color.kWhite);
              }
            }
          }
          m_led.setData(m_buffer);
        })
        .ignoringDisable(true);
  }
}
