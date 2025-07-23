package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
  public static final class HardwareConstants {
    public static final int LED_PORT = 1;
    public static final int LED_LENGTH = 25;
  }

  public static final class PatternConfig {
    public static final LEDPattern LED_POSED_SOLID = LEDPattern.solid(new Color(0, 255, 0));

    // Drivetrain posing or Shooter revving
    public static final LEDPattern LED_POSING_BLINK = LEDPattern.solid(Color.kYellow);
    public static final double LED_POSING_BLINK_SPEED = 0.075;

    // VISION searching for target
    public static final LEDPattern LED_VISION_SEARCHING_BLINK = LEDPattern.solid(Color.kWhite);
    public static final double LED_VISION_BLINKING_SEARCH_SPEED = 0.075;

    // lg ground intake
    public static final LEDPattern LED_ALGAE_GROUND_BREATH =
        LEDPattern.solid(new Color(0, 255, 65));
    public static final double LED_ALGAE_GROUND_BREATH_SPEED = 2;

    // lg arm intake
    public static final LEDPattern LED_ALGAE_PINCHER_BLINK =
        LEDPattern.solid(new Color(0, 255, 65));
    public static final double LED_ALGAE_PINCHER_BLINK_SPEED = 0.075;

    // Coral Alligned
    public static final LEDPattern LED_CORAL_ALIGNED_BLINK =
        LEDPattern.solid(new Color(212, 0, 255));
    public static final double LED_CORAL_ALIGNED_BLINK_SPEED = 0.075;

    // coral photoeletric detects coral
    public static final LEDPattern LED_CORAL_DETECTED_BREATH = LEDPattern.solid(Color.kPurple);
    public static final double LED_CORAL_DETECTED_BREATH_SPEED = 2.0;

    // coral intake running
    public static final LEDPattern LED_CORAL_DETECTED_BLINK = LEDPattern.solid(Color.kPurple);
    public static final double LED_CORAL_DETECTED_BLINK_SPEED = 0.075;

    // robot idle (no status) - breathing pattern
    public static final LEDPattern LED_NOSTATUS_BREATH = LEDPattern.solid(Color.kRed);

    public static final double LED_NOSTATUS_BREATH_SPEED = 25;

    // robot disabled - scrolling pattern
    public static final LEDPattern LED_DISABLED_SCROLL =
        LEDPattern.gradient(
            LEDPattern.GradientType.kContinuous, new Color(255, 130, 0), new Color(200, 80, 10));

    public static final double LED_DISABLED_SCROLL_SPEED = 2;
    // test
    public static final double LED_RAINBOW_SCROLL_SIZE = 1.0; // 0.0 - 1.0
    public static final int LED_RAINBOW_SCROLL_SPEED = 150;
    public static final LEDPattern LED_PROGRESS_GRADIENT_PATTERN =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
    // public static final double kLEDBreatheSpeedSlow = 2;

    public static final LEDPattern SHOOTER_NOT_READY = LEDPattern.solid(Color.kYellow);
    public static final double SHOOTER_NOT_READY_BLINK_SPEED = 0.075;

    public static final LEDPattern SHOOTER_READY =
        LEDPattern.solid(
            new Color(
                13, 225,
                13)); // LEDPattern.gradient(GradientType.kContinuous, new Color(13, 225, 13), new
    // Color(129, 229, 129));
    public static final double SHOOTER_READY_BLINK_SPEED = 2.0;

    public static final LEDPattern APRIL_TAGS_COLOR = LEDPattern.solid(Color.kWhite);
    public static final double APRIL_TAGS_SPEED = 0.075;
  }
}
