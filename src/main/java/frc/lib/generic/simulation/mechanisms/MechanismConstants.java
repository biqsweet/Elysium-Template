package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class MechanismConstants {
    protected static int
            DEFAULT_ROOT_X = 5,
            DEFAULT_ROOT_Y = 5,
            DEFAULT_CANVAS_WIDTH = 10,
            DEFAULT_CANVAS_HEIGHT = 10,
            DEFAULT_LINE_WIDTH = 3;

    protected static double
            DEFAULT_ARROW_TIP1 = 135,
            DEFAULT_ARROW_TIP2 = -135,
            DEFAULT_ARROW_TIP1_INVERSE = 45,
            DEFAULT_ARROW_TIP2_INVERSE = -45;

    protected static Color8Bit
            RED = new Color8Bit(Color.kRed),
            GREEN = new Color8Bit(Color.kGreen),
            BLUE = new Color8Bit(Color.kBlue),
            GRAY = new Color8Bit(Color.kGray),
            DARK_BLUE = new Color8Bit(Color.kDarkBlue),
            DARK_GRAY = new Color8Bit(Color.kDarkGray);
}
