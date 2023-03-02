// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.simulation;

import java.util.ArrayList;
import java.util.Random;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.framework.mechanismsAdvanced.SwerveModule;
import frc.robot.framework.motors.Motor;

/** Add your docs here. */
public class Simulator {

    private static final Simulator sim = new Simulator();
    private final ArrayList<SimProfile> simProfiles = new ArrayList<SimProfile>();

    /**
     * Gets the robot simulator instance.
     */
    public static Simulator getInstance() {
        return sim;
    }

    public void addSparkMax(CANSparkMax sparkMax, DCMotor dcMotor){
        REVPhysicsSim.getInstance().addSparkMax(sparkMax, dcMotor);
    }

    public void addCANCoder(
            WPI_CANCoder wpi_CANCoder,
            Motor motor,
            double motorToMechConversion) {

        if (wpi_CANCoder != null) {
            SimDeviceSim sim = new SimDeviceSim("CANEncoder:CANCoder[" + wpi_CANCoder.getDeviceID() + "]");
            SimDouble simPosition = new SimDouble(
                    SimDeviceDataJNI.getSimValueHandle(sim.getNativeHandle(), "rawPositionInput"));

            Random random = new Random();
            int randomStart = (int) (random.nextDouble() * 4096.0);
            simPosition.set(randomStart);

            SimProfile canCoderSimProfile = new SimProfile() {

                @Override
                public void run() {
                    double deltaT = getPeriod();
                    double currentAngleDegrees = simPosition.get() * 360.0 / 4096.0;
                    double deltaAngleDegrees = motor.getVelocityRotationPerSecond().getDegrees() * motorToMechConversion
                            * deltaT;
                    currentAngleDegrees += deltaAngleDegrees;
                    double currentAngleRotations = currentAngleDegrees / 360.0;
                    double currentAngle1Rotation = currentAngleRotations %= 1.0;
                    double currentAngleNative = currentAngle1Rotation * 4096.0;
                    int currentAngleNativeInt = (int) currentAngleNative;
                    simPosition.set(currentAngleNativeInt);
                }
            };
            simProfiles.add(canCoderSimProfile);
        }
    }

    public void addREVThroughBoreEncoder(
            DutyCycleEncoder revThroughBoreEncoder,
            double offSetDegrees,
            Motor motor,
            double motorToMechConversion) {

        if (revThroughBoreEncoder != null) {
            SimDeviceSim sim = new SimDeviceSim(
                    "DutyCycle:DutyCycleEncoder",
                    revThroughBoreEncoder.getSourceChannel());
            SimDouble simPosition = new SimDouble(
                    SimDeviceDataJNI.getSimValueHandle(
                            sim.getNativeHandle(),
                            "absPosition"));

            Random random = new Random();
            double randomStart = random.nextDouble() * (180 - 16) + offSetDegrees;
            randomStart += 7;
            randomStart /= 360;
            simPosition.set(randomStart);

            SimProfile revThroughBoreEncoderSimProfile = new SimProfile() {

                @Override
                public void run() {
                    double deltaT = getPeriod();
                    double currentAngleDegrees = simPosition.get() * 360.0;
                    double deltaAngleDegrees = motor.getVelocityRotationPerSecond().getDegrees() * motorToMechConversion
                            * deltaT;
                    currentAngleDegrees += deltaAngleDegrees;
                    double currentAngleRotations = currentAngleDegrees / 360.0;
                    double currentAngle1Rotation = currentAngleRotations %= 1.0;
                    currentAngle1Rotation = currentAngle1Rotation < 0 ? currentAngle1Rotation + 1.0
                            : currentAngle1Rotation;
                    simPosition.set(currentAngle1Rotation);
                }
            };
            simProfiles.add(revThroughBoreEncoderSimProfile);
        }
    }

    public void addPWFTimeOfFlightSensor(
            int sensorId,
            Motor motor,
            double motorToMechConversion,
            double rotationtoLinearConversion,
            double minimumPositionMeters,
            double maximumPositionMeters) {

        SimDeviceSim simTimeOfFlight = new SimDeviceSim("PWF:TimeOfFlight", sensorId);
        SimDouble simRange = new SimDouble(
                SimDeviceDataJNI.getSimValueHandle(simTimeOfFlight.getNativeHandle(), "Range (mm)"));

        Random random = new Random();
        double maxMinDifference = (maximumPositionMeters - minimumPositionMeters) * 1000;
        double simStart = random.nextDouble() * maxMinDifference + minimumPositionMeters * 1000;
        simRange.set(simStart);

        SimProfile pwfTimeOfFlightSimProfile = new SimProfile() {

            @Override
            public void run() {
                if (motor != null) {
                    double currentPositionMillimeters = simRange.get();
                    double deltaT = getPeriod();
                    double linearRate = motor.getVelocityRotationPerSecond().getRotations() * motorToMechConversion
                            * rotationtoLinearConversion * 1000;
                    double deltaPos = linearRate * deltaT;
                    double newPos = currentPositionMillimeters + deltaPos;
                    simRange.set(newPos);
                }
                else{
                    simRange.set(Double.NaN);
                }
            }
        };

        simProfiles.add(pwfTimeOfFlightSimProfile);
    }

    public void addSwerveModules(
            AHRS ahrs,
            SwerveDriveKinematics swerveDriveKinematics,
            SwerveModule... swerveModules) {

        if (ahrs != null) {
            SwerveModuleState[] swerveModuleStates = new SwerveModuleState[swerveModules.length];

            Supplier<Rotation2d> yawRateSupplier = () -> {
                for (int i = 0; i < swerveModuleStates.length; i++) {
                    swerveModuleStates[i] = swerveModules[i].getSwerveModuleState();
                }
                ChassisSpeeds chassisSpeeds = swerveDriveKinematics.toChassisSpeeds(swerveModuleStates);

                return Rotation2d.fromRadians(-chassisSpeeds.omegaRadiansPerSecond);
            };

            int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            SimDouble simulatedYawDegrees = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
            simulatedYawDegrees.set(0.0);

            SimProfile navxSim = new SimProfile() {

                @Override
                public void run() {
                    double deltaT = getPeriod();
                    double currentSimulatedYawDegrees = simulatedYawDegrees.get();

                    double deltaYaw = yawRateSupplier.get().getDegrees() * deltaT;

                    currentSimulatedYawDegrees += deltaYaw;

                    currentSimulatedYawDegrees %= 360;

                    if (currentSimulatedYawDegrees > 180) {
                        currentSimulatedYawDegrees -= 360;
                    } else if (currentSimulatedYawDegrees <= -180) {
                        currentSimulatedYawDegrees += 360;
                    }

                    simulatedYawDegrees.set(currentSimulatedYawDegrees);

                }

            };

            simProfiles.add(navxSim);

        }
    }

    public void run() {
        // Simulate devices
        for (SimProfile simProfile : simProfiles) {
            simProfile.run();
        }
    }
}
