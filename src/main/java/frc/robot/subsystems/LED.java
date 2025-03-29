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
import java.util.List;
import java.util.Random;

public class LED extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 18;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  // Fields for blue fire simulation
  private final List<Flame> flames = new ArrayList<>();
  private final int simulationBarLength = kLength; // Use LED strip length for simulation
  private final int notActuallyMaxIntensity = 16;
  private final Random random = new Random();

  public LED() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to netchecker (which now uses our blue fire simulation)
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
  private static final double SNOW_START_POSITION = 19.0; // Where new snow spawns

  // Bottom stack constants
  private static final double BOTTOM_STACK_SHIFT_FACTOR = -0.25; // Proportional push term
  private static final double INITIAL_BOTTOM_STACK_POS = 3.0; // Starting stack height

  // Falling snow constants
  private static final double FLAKE_RANDOM_SPEED = -40; // Random part of velocity
  private static final double FLAKE_BASE_SPEED = 5; // Base velocity

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
      nextSnowflakeDelay = (Math.pow(Math.random(), 2.5) * 2) + 0.4;
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
              // Snow:
              // snowPattern().reversed().applyTo(m_buffer);
              // Blue fire simulation:
              // • Create a new flame and add it to the simulation.
              // • Update all flames (each flame’s length and position evolve over time).
              // • Remove any flames that have “died.”
              // • Build a fire-intensity list (one intensity value per LED).
              // • Map the intensity values into RGB colors for each LED.
              Flame newFlame = buildFlame(simulationBarLength);
              flames.add(newFlame);

              updateFlames(flames, simulationBarLength);
              flames.removeIf(flame -> flame.getLength() <= 0);

              List<Integer> fires = new ArrayList<>();
              for (int i = 0; i < simulationBarLength; i++) {
                fires.add(0);
              }
              makeFireList(fires, flames);
              updateLEDs(fires, notActuallyMaxIntensity);
            }
          }
          // Update the LED strip with the new data.
          m_led.setData(m_buffer);
        })
        .ignoringDisable(true);
  }

  // --- Blue Fire Simulation Methods ---

  /**
   * Creates a new flame for the simulation.
   *
   * @param barLength the length of the simulation (number of LEDs)
   * @return a new Flame instance with randomized length
   */
  private Flame buildFlame(int barLength) {
    int num = Math.round((float) (0.9 * barLength));
    int length1 = random.nextInt(num) + 1;
    int length = Math.round((float) (length1 * length1 / num));
    return new Flame(length, 0);
  }

  /**
   * Updates each flame’s length and position to simulate fire evolution.
   *
   * @param flames the list of flames to update
   * @param barLength the total number of LEDs (simulation width)
   */
  private void updateFlames(List<Flame> flames, int barLength) {
    for (Flame flame : flames) {
      int length = flame.getLength();
      int position = flame.getPosition();

      int lengthNum = random.nextInt(2) + 1;
      if (lengthNum == 1) {
        length = length - 1;
      }
      int posNum = random.nextInt(4) + 1;
      if (posNum == 1) {
        position++;
      }
      if (position + length >= barLength) {
        length = length - 1;
      }
      flame.setLength(length);
      flame.setPosition(position);
    }
  }

  /**
   * Aggregates the contributions from each flame into a fire intensity list.
   *
   * @param fires a list of intensity values (one per LED) that will be updated
   * @param flames the list of flames contributing to the intensity
   */
  private void makeFireList(List<Integer> fires, List<Flame> flames) {
    for (Flame flame : flames) {
      int length = flame.getLength();
      int position = flame.getPosition();
      for (int i = 0; i < length; i++) {
        int idx = position + i;
        if (idx < fires.size()) {
          fires.set(idx, fires.get(idx) + 1);
        }
      }
    }
  }

  /**
   * Updates the LED buffer colors based on the fire intensity values.
   *
   * @param fires the list of fire intensity values (one per LED)
   * @param maxIntensity the maximum expected intensity used for scaling
   */
  private void updateLEDs(List<Integer> fires, int maxIntensity) {
    for (int i = 0; i < fires.size(); i++) {
      int fire = fires.get(i);
      if (fire > maxIntensity) {
        // When intensity is very high, use a bright blue color.
        m_buffer.setLED(i, new Color(100, 200, 255));
      } else {
        int red = Math.round(fire * 75 / (float) maxIntensity);
        int green = Math.round(fire * 135 / (float) maxIntensity);
        int blue = Math.round(fire * 255 / (float) maxIntensity);
        m_buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  // --- Inner Class for Flame Simulation ---
  private static class Flame {
    private int length;
    private int position;

    public Flame(int length, int position) {
      this.length = length;
      this.position = position;
    }

    public int getLength() {
      return length;
    }

    public int getPosition() {
      return position;
    }

    public void setLength(int length) {
      this.length = length;
    }

    public void setPosition(int position) {
      this.position = position;
    }
  }
}
