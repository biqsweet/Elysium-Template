package frc.robot.subsystems.shooter;

import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class ShooterConstants {
    protected static final Motor SHOOTER_MOTOR = MotorFactory.createSpark("CONVEYOR_MOTOR", 102, MAX);
    protected static final Motor SHOOTER_TURNING_MOTOR = MotorFactory.createSpark("CONVEYOR_MOTOR", 103, MAX);
    // The shooter needs to turn
    // I think it's like the turning of the swerve but how do you control it
    static final MotorProperties.IdleMode SHOOTER_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final MotorConfiguration conveyorMotorConfiguration = new MotorConfiguration();

    static {
        configureIntakeConfiguration();
    }

    private static void configureIntakeConfiguration() {
        conveyorMotorConfiguration.idleMode = SHOOTER_NEUTRAL_MODE;
    }
}

