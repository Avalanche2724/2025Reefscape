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
import java.util.ArrayList;

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
    updateSnow();
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

  // Example constants you can tweak
  private static final double TIME_STEP = 0.02; // Called ~50x/sec if periodic
  private static final double SNOW_START_POSITION = 18.0; // Where new snow spawns

  // Bottom stack constants
  private static final double BOTTOM_STACK_SHIFT_FACTOR = -0.25; // Proportional push term
  private static final double INITIAL_BOTTOM_STACK_POS = 3.0; // Starting stack height

  // Falling snow constants
  private static final double FLAKE_RANDOM_SPEED = -20; // Random part of velocity
  private static final double FLAKE_BASE_SPEED = 3; // Base velocity

  public ArrayList<Double> snowPositions = new ArrayList<>();
  public double bottomSnowStackPosition = INITIAL_BOTTOM_STACK_POS;

  public double timeSinceLastSnowflake = 1.5;
  public double nextSnowflakeDelay = 0;

  /** Update positions of falling snowflakes and bottom snow stack. */
  public void updateSnow() {
    timeSinceLastSnowflake += TIME_STEP;

    // 1) Spawn a new snowflake if we've passed the current delay
    if (timeSinceLastSnowflake >= nextSnowflakeDelay) {
      snowPositions.add(SNOW_START_POSITION);
      // Reset timer
      timeSinceLastSnowflake = 0.0;
      // Pick new random delay
      nextSnowflakeDelay = Math.pow((Math.random() + 1), 1.5) - 0.5;
    }

    // 2) Update bottom snow stack position
    bottomSnowStackPosition += (BOTTOM_STACK_SHIFT_FACTOR * TIME_STEP * bottomSnowStackPosition);

    // 3) Push all falling snowflakes down
    //    This is effectively: pos = pos + randomTerm - baseTerm,
    //    but note that FLAKE_RANDOM_SPEED and FLAKE_BASE_SPEED are negative,
    //    so it ends up going downward in your coordinate system.
    snowPositions.replaceAll(
        pos ->
            pos
                + (Math.random() * FLAKE_RANDOM_SPEED * TIME_STEP)
                + (FLAKE_BASE_SPEED * TIME_STEP));

    // 4) Remove any flakes that have collided with the bottom stack
    while (!snowPositions.isEmpty() && (snowPositions.get(0) - 1) < bottomSnowStackPosition) {
      snowPositions.remove(0);
      bottomSnowStackPosition += 1.0;
    }
  }

  static final double GAMMA = Robot.isReal() ? 2.2 : 1.0;

  private int gammaCorrect(int value) {
    // Convert [0..255] to [0..1], raise to 1/gamma, then convert back.
    double normalized = value / 255.0;
    double corrected = Math.pow(normalized, GAMMA);
    return (int) (corrected * 255.0 + 0.5);
  }

  public LEDPattern snowPattern() {
    return (reader, writer) -> {
      int length = reader.getLength();
      if (length <= 1) {
        return; // Edge case: no LEDs
      }

      // We'll assume LED i=0 is at "top"=18.0, and LED i=(length-1) is "bottom"=0.0
      // If your physical strip is reversed, just invert the mapping below.

      for (int i = 0; i < length; i++) {
        // ----------------------------------------------------------------------
        // 1) Map this LED index to a "physical" position in [18 .. 0].
        // ----------------------------------------------------------------------
        double ledPosition = 18.0 - (18.0 * i / (length - 1.0));

        // ----------------------------------------------------------------------
        // 2) Compute tStack in [0..1] for bottom stack boundary
        //    If ledPosition <= bottomSnowStackPosition => fully white (1.0)
        //    Otherwise fade from 1..0 over some radius above that.
        // ----------------------------------------------------------------------
        double distFromStack = ledPosition - bottomSnowStackPosition;
        double stackRadius = 1.0; // Tune this for a sharper or softer boundary
        double tStack;
        if (distFromStack <= 0) {
          // LED is at or below the snow stack => fully white
          tStack = 1.0;
        } else {
          // If within stackRadius above the boundary, fade up to white
          tStack = 1.0 - (distFromStack / stackRadius);
        }
        if (tStack < 0) tStack = 0;
        if (tStack > 1) tStack = 1;

        // ----------------------------------------------------------------------
        // 3) Compute tFlake in [0..1] for nearest falling snowflake
        // ----------------------------------------------------------------------
        double minDistFlake = Double.POSITIVE_INFINITY;
        if (!snowPositions.isEmpty()) {
          for (double snowY : snowPositions) {
            double dist = Math.abs(ledPosition - snowY);
            if (dist < minDistFlake) {
              minDistFlake = dist;
            }
          }
        } else {
          // If no flakes, minDistFlake remains ∞ => tFlake = 0 => no whiteness from flakes
        }

        double flakeRadius = 1.0; // Tune for how far from a flake you get whiteness
        double tFlake = 1.0 - (minDistFlake / flakeRadius);
        if (tFlake < 0) tFlake = 0;
        if (tFlake > 1) tFlake = 1;

        // ----------------------------------------------------------------------
        // 4) Final whiteness factor is the max of stack or flake
        // ----------------------------------------------------------------------
        double t = Math.max(tStack, tFlake);

        // ----------------------------------------------------------------------
        // 5) Interpolate from blue -> white without allocating new Color objects
        //    Blue(0,0,255) to White(255,255,255)
        //      R channel: 0   -> 255
        //      G channel: 0   -> 255
        //      B channel: 255 -> 255
        // ----------------------------------------------------------------------
        // clamp t into [0..1] just for safety
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        // Convert to 8-bit color channels
        int r = (int) (255 * t); // from 0 to 255
        int g = (int) (255 * t); // from 0 to 255
        int b = 255; // always 255 (blue channel)

        // Now set the LED color (no new object allocated)
        writer.setRGB(i, gammaCorrect(r), gammaCorrect(g), gammaCorrect(b));
      }
    };
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
              snowPattern().reversed().applyTo(m_buffer);
              // Create a base alternating pattern: even indices are white, odd indices are blue.
              /*
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
              */
            }
          }
          // Update the LED strip with the new data.
          m_led.setData(m_buffer);
        })
        .ignoringDisable(true);
  }
}
