package frc.robot.subsystems.intake;

import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class IntakeConstants {
    protected static final Motor INTAKE_MOTOR = MotorFactory.createSpark("INTAKE_MOTOR", 100, MAX);

    static final MotorProperties.IdleMode INTAKE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();

    static{
        configureIntakeConfiguration();
    }
    private static void configureIntakeConfiguration() {
        intakeMotorConfiguration.idleMode = INTAKE_NEUTRAL_MODE;
    }

}
