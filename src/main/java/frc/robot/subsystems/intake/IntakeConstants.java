package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class IntakeConstants {
    protected static final Motor INTAKE_MOTOR = MotorFactory.createSpark("INTAKE_MOTOR", 100, MAX);

    static {
        configureIntakeMotor();
    }

    private static void configureIntakeMotor() {
        MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();

        intakeMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        intakeMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 150, 0.02);


        INTAKE_MOTOR.configure(intakeMotorConfiguration);
    }

}
