package frc.robot.Subsystems.LEDs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Climb.ClimbConstants;
import java.util.Map;

public class LEDs extends SubsystemBase {

  private int port;
  private int length;
  AddressableLED led;
  AddressableLEDBuffer buffer;

  public LEDs(int port, int length) {
    this.port = port;
    this.length = length;

    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);

    led.setLength(length);
    led.start();
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(buffer);
  }

  public void rainbowScroll() {

    Map<Double, Color> maskSteps =
        Map.of(0.0, Color.kWhite, LEDConstants.PatternConfig.LED_RAINBOW_SCROLL_SIZE, Color.kBlack);
    LEDPattern base = LEDPattern.rainbow(255, 255);
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtRelativeSpeed(
                Percent.per(Second).of(LEDConstants.PatternConfig.LED_RAINBOW_SCROLL_SPEED));

    LEDPattern pattern = base.mask(mask);
    // pattern = base.mask(progress);

    setPattern(pattern);
  }

  public void climbProgress() {
    double percent =
        RobotContainer.m_Climb.getPosition() / ClimbConstants.ControlConstants.climberUpPosition;
    LEDPattern progress = LEDPattern.progressMaskLayer(() -> percent);
    LEDPattern base = LEDConstants.PatternConfig.LED_NOSTATUS_BREATH;
    LEDPattern pattern = base.mask(progress);

    setPattern(pattern);
  }

  public void scroll(LEDPattern pattern, double speed) {
    // SmartDashboard.putString(getName(), "scroll");
    // Logger.recordOutput(getName(), "scroll");
    setPattern(pattern.scrollAtRelativeSpeed(Percent.per(Second).of(speed)));
  }

  public void blink(LEDPattern pattern, double speed) {
    // SmartDashboard.putString(getName(), "blink");
    // Logger.recordOutput(getName(), "blink");
    setPattern(pattern.blink(Seconds.of(speed)));
  }

  public void breathe(LEDPattern pattern, double speed) {
    // SmartDashboard.putString(getName(), "breathe");
    // Logger.recordOutput(getName(), "breathe");
    setPattern(pattern.breathe(Second.of(speed)).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }

  public void progress(LEDPattern mask, double percentage) {
    // SmartDashboard.putString(getName(), "progress");
    // Logger.recordOutput(getName(), "breathe");
    LEDPattern base = LEDPattern.progressMaskLayer(() -> percentage);
    setPattern(base.mask(mask));
  }

  public void solid(LEDPattern pattern) {
    // SmartDashboard.putString(getName(), "Solid");
    // Logger.recordOutput(getName(), "Solid");
    setPattern(pattern);
  }

  @Override
  public void periodic() {
    // testing inputs change later

    // Logger.recordOutput("LED/Buffer", pa);

    if (DriverStation.isDisabled()) { // disabled
      scroll(
          LEDConstants.PatternConfig.LED_DISABLED_SCROLL,
          LEDConstants.PatternConfig.LED_DISABLED_SCROLL_SPEED);
    } else if (RobotContainer.m_Climb.getPosition() >= 0.05) {
      rainbowScroll();
    }
    // climbing
    // else if(RobotContainer.m_climber.isClimbing()){
    //   rainbowScroll();
    // }
    // coral loaded
    else if (RobotContainer.m_EndEffector.getCoralInput()) {
      blink(
          LEDConstants.PatternConfig.LED_CORAL_DETECTED_BLINK,
          LEDConstants.PatternConfig.LED_CORAL_DETECTED_BLINK_SPEED);
    }
    // coral station ready
    /* else if (RobotContainer.m_EndEffector.getCoralInput()){
            && RobotContainer.lime.isTagReached()) {
        blink(
                Constants.LEDConstants.PatternConfig.kLEDCoralAllignedBlink,
                Constants.LEDConstants.PatternConfig.kLEDCoralAllignedBlinkSpeed);
    }
    // Aligned to barge
    else if (RobotContainer.bargeTagStatsLimelight.isTagReached()) {
        solid(Constants.LEDConstants.PatternConfig.kShooterIsReady);

    }
    // Tracking April Tag
<<<<<<< Updated upstream
    else if (RobotContainer.bargeTagStatsLimelight.isTagDetected()) {
        breathe(
                Constants.LEDConstants.PatternConfig.kAprilTags,
                Constants.LEDConstants.PatternConfig.kAprilTagBlinkSpeed);
    } else if (RobotContainer.m_groundIntake.getAlgaeDetected()) { // algae ground intake/hold
        breathe(
                Constants.LEDConstants.PatternConfig.kLEDAlgaeGroundBreathe,
                Constants.LEDConstants.PatternConfig.kLEDAlgaeGroundBreatheSpeed);
=======
    else if (RobotContainer.m_Limelight.hasTargets(
            LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY)
        && RobotContainer.driverController.getLeftTriggerAxis() > 0.5) {
      breathe(LEDPattern.solid(Color.kWhite), 0.075);
    }
    // Align Complete
    else if (LimelightIO.isAligned) {
      blink(
          LEDConstants.PatternConfig.LED_POSED_BLINK,
          LEDConstants.PatternConfig.LED_POSED_BLINK_SPEED);
    } /*else if (RobotContainer.m_groundIntake.getAlgaeDetected()) { // algae ground intake/hold
          breathe(
                  Constants.LEDConstants.PatternConfig.kLEDAlgaeGroundBreathe,
                  Constants.LEDConstants.PatternConfig.kLEDAlgaeGroundBreatheSpeed);
>>>>>>> Stashed changes

    } */
    else if (RobotContainer.m_EndEffector.getAlgaeInput()) { // algae end effector
      blink(
          LEDConstants.PatternConfig.LED_ALGAE_PINCHER_BLINK,
          LEDConstants.PatternConfig.LED_ALGAE_PINCHER_BLINK_SPEED);
    } else if (RobotContainer.m_GroundIntake.getAlgaeDetected()) {
      breathe(
          LEDConstants.PatternConfig.LED_ALGAE_GROUND_BREATH,
          LEDConstants.PatternConfig.LED_ALGAE_GROUND_BREATH_SPEED);
    }

    // } else if (!RobotContainer.m_endEffector.getCoralDigitalInput().get()) { // coral detected
    //   breathe(
    //       Constants.LEDConstants.PatternConfig.kLEDCoralDetectedBreathe,
    //       Constants.LEDConstants.PatternConfig.kLEDCoralDetectedBreatheSpeed);
    // } else if (RobotContainer.m_endEffector.motorRunning()) { //?
    //   blink(
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlink,
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlinkSpeed); //supposed to be
    // coral motor running
    // } else if (RobotContainer.m_endEffector.motorRunning()) { //?
    //   blink(
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlink,
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlinkSpeed); //supposed to be
    // coral motor running
    //  }
    else { // gold breathe idle
      scroll(LEDConstants.PatternConfig.LED_NOSTATUS_BREATH, 50);
    }

    led.setData(buffer);
  }
}
