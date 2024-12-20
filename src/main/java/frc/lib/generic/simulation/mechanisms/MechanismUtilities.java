package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.DEFAULT_ROOT_X;
import static frc.lib.generic.simulation.mechanisms.MechanismConstants.DEFAULT_ROOT_Y;


public class MechanismUtilities {
    protected static MechanismRoot2d getRoot(String name, Mechanism2d mechanism) {
        return mechanism.getRoot(name, DEFAULT_ROOT_X, DEFAULT_ROOT_Y);
    }
}
