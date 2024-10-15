package frc.robot.subsystems.arm;

import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class ArmConstants {
    protected static final Motor ARM_MOTOR = MotorFactory.createSpark("ARM_MOTOR", 102, MAX);
    protected static final Encoder ARM_ENCODER = EncoderFactory.createCanCoder("ARM_ENCODER", 103);


    static {
        configureMotorConfiguration();
    }

    private static void configureMotorConfiguration() {
        MotorConfiguration armMotorConfiguration = new MotorConfiguration();

        armMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

//        armMotorConfiguration.simulationSlot = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM,);
        //SimulationProperties.Slot needs:
        // Simulation Type    ARM
        // DC Motor gearbox   ???
        // double gear Ratio  ???
        // double armLength Meters ???
        // double momentOfInertia ???
        // Rotation2d minimumAngle ???
        // Rotation2d maximumAngle ???
        // boolean simulateGravity ???

    }
}

