package frc.utils.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.LEDStrip;
import frc.utils.Color.RGB;

/**
 * A wave pattern that can be applied to an LED strip.
 * This pattern will interpolate between the primary and secondary colors.
 * These colors will shift over time.
 * @see frc.team4481.lib.feedback.led.LEDStrip
 */
public class WavePattern implements LEDPattern {
    /**
     * Updates the LED buffer with the pattern.
     *
     * @param buffer The LED buffer to update.
     * @param strip  The strip to update the buffer with.
     */
    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
        int offset = strip.getOffset();
        int length = strip.getLength();
        RGB primary = strip.getPrimaryColor().getRGB();
        RGB secondary = strip.getSecondaryColor().getRGB();

        // Fading is done by changing the value of the color over a sine wave
        double ledWavelength = (2 * Math.PI) / length;
        double timeWavelength = (2 * Math.PI) / strip.getPatternDuration();

        // Calculate time offset
        double timeOffset = Timer.getFPGATimestamp() * timeWavelength;

        for (int i = 0; i < length; i++) {
            double interpolation = Math.sin(ledWavelength * i + timeOffset) * 0.5 + 0.5;

            buffer.setRGB(
                    offset + i,
                    interpolate(primary.red(), secondary.red(), interpolation),
                    interpolate(primary.green(), secondary.green(), interpolation),
                    interpolate(primary.blue(), secondary.blue(), interpolation)
            );
        }
    }

    private int interpolate(int start, int end, double interpolationValue) {
        return (int) (start + (end - start) * interpolationValue);
    }
}
