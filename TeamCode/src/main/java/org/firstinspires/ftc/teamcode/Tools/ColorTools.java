package org.firstinspires.ftc.teamcode.Tools;


import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.LinkedList;
import java.util.Queue;

/**
 * @author Lee
 */
public class ColorTools {
    int colorHSVnow;
    int listSize;
    int averageLastHue;
    int total;
    float[] hsvNow;
    Queue<Integer> averageList;
    final double SCALE_FACTOR = 255; // Umrechnungsfaktor

    boolean isColorChanged;

    public boolean isColor(ColorEnum color_enum, ColorSensor colorSensor) {
        if (color_enum.equals(ColorEnum.Blue)) return isBlue(colorSensor);
        return isRed(colorSensor);
    }

    /**
     * it returns true if the color is red
     * red is between 0-60° or 330-360° (hue) and with saturation over 0.3
     *
     * @param colorSensor is the color sensor we get our values from
     * @return it true or false
     */
    public boolean isRed(ColorSensor colorSensor) {
        float[] hsvValues = showHSV(colorSensor);

        if (hsvValues[0] >= 0 && hsvValues[0] <= 60 && hsvValues[1] >= 0.27) {
            return true;
        } else if (hsvValues[0] >= 330 && hsvValues[0] <= 360) {
            return true;
        }

        return false;
    }

    /**
     * it returns true if the color is blue
     * blue is between 120-290° (hue) and with saturation over 0.3
     *
     * @param colorSensor is the color sensor we get our values from
     * @return true or false
     */
    public boolean isBlue(ColorSensor colorSensor) {
        float[] hsvValues = showHSV(colorSensor);

        if (hsvValues[0] >= 160 && hsvValues[0] <= 290  && hsvValues[1] >= 0.27 ) { //[0] 120, && hsvValues[1] >= 2 &&  && hsvValues[2] <= 7
            return true;
        }

        return false;
    }

    public boolean isWhite(ColorSensor colorSensor) {
        float[] hsvValues = showHSV(colorSensor);

        if (hsvValues[1] < 0.1 && hsvValues[2] > 150) {
            return true;
        }

        return false;
    }


    /**
     * the boolean gives a true back if the color has changed
     * it calculates the average of the queue and compares it to the average before
     *
     * @param colorSenseChange is the color sensor we get our values from
     * @return true or false
     */

    public boolean colorChange(ColorSensor colorSenseChange, Queue<Integer> colorList) {
        averageList = new LinkedList<>();
        hsvNow = showHSV(colorSenseChange);
        colorHSVnow = (int) hsvNow[0];
        listSize = colorList.size();
        averageLastHue = 0;
        total = 0;
        isColorChanged = false;

        for (int hue : colorList) {
            total = total + hue;
        }

        averageLastHue = total / 100;

        if ((averageLastHue < colorHSVnow - 10 || averageLastHue > colorHSVnow + 10)) {
            isColorChanged = true;
        }

        return isColorChanged;
    }

    /**
     * it converts rgb to hsv-values
     *
     * @param HSVvalues is the color sensor we get the rgb-values from
     * @return it returns hsv-values
     */
    public float[] showHSV(ColorSensor HSVvalues) {
        float[] hsv = new float[3];

        Color.RGBToHSV((int) (HSVvalues.red() * SCALE_FACTOR),
                (int) (HSVvalues.green() * SCALE_FACTOR),
                (int) (HSVvalues.blue() * SCALE_FACTOR),
                hsv);

        return hsv;
    }
}

