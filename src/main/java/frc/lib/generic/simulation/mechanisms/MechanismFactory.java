package frc.lib.generic.simulation.mechanisms;

import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class MechanismFactory {
    /**
     * @return SingleJointedArmMechanism2d if CURRENT_MODE is simulation, null if it isn't
     * @see SingleJointedArmMechanism2d
     */
    public static SingleJointedArmMechanism2d createSingleJointedMechanism(String name, double armLength) {
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SingleJointedArmMechanism2d(name, armLength);
        }
        return null;
    }

    /**
     * @return DoubleJointedArmMechanism2d if CURRENT_MODE is simulation, null if it isn't
     * @see DoubleJointedArmMechanism2d
     */
    public static DoubleJointedArmMechanism2d createDoubleJointedMechanism(String name, double shoulderLength, double elbowLength) {
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new DoubleJointedArmMechanism2d(name, shoulderLength, elbowLength);
        }
        return null;
    }

    /**
     * @return SpeedMechanism2d if CURRENT_MODE is simulation, null if it isn't
     * @see SpeedMechanism2d
     */
    public static SpeedMechanism2d createSpeedMechanism(String name) {
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SpeedMechanism2d(name);
        }
        return null;
    }

    /**
     * @return ElevatorMechanism2d if CURRENT_MODE is simulation, null if it isn't
     * @see ElevatorMechanism2d
     */
    public static ElevatorMechanism2d createElevatorMechanism(String name, double elevatorLength) {
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new ElevatorMechanism2d(name, elevatorLength);
        }
        return null;
    }

    /**
     * @return ArmElevatorMechanism2d if CURRENT_MODE is simulation, null if it isn't
     * @see ArmElevatorMechanism2d
     */
    public static ArmElevatorMechanism2d createArmElevatorMechanism(String name, double armLength) {
        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new ArmElevatorMechanism2d(name, armLength);
        }
        return null;
    }
}
