package frc.robot.subsystems.verticalconveyor;

import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class VerticalConveyorConstants {
    protected static final Motor CONVEYOR_MOTOR = MotorFactory.createSpark("CONVEYOR_MOTOR", 101, MAX);

    static final MotorProperties.IdleMode CONVEYOR_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final MotorConfiguration conveyorMotorConfiguration = new MotorConfiguration();

    static {
        configureConveyorConfiguration();
    }

    private static void configureConveyorConfiguration() {
        conveyorMotorConfiguration.idleMode = CONVEYOR_NEUTRAL_MODE;
    }
}