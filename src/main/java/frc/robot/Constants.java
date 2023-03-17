// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.framework.motors.Motor;
import frc.robot.subsystems.ElevatorTilt;
import frc.robot.subsystems.ElevatorTilt.State;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final class DriverController {
      public static final int kDriverControllerPort = 0;
      public static final double kDeadband = 0.1;
      public static final double maxTranslationalSpeedMPS = 6.0;
      public static final double maxRotationalSpeedRadPS = 4.0;
      public static final int kOperatorControllerPort = 1;
    }

    public static final class ElevatorManualControls {
      public static final double kDeadband = 0.1;
    }

    public static final class OperatorController {
      public static final int kDriverControllerPort = 1;
    }

    public static final class TertiaryController {
      public static final int kDriverControllerPort = 2;
    }
  }

  public static class Robot {
    public static class Drive {
      public static final double robotLength = Units.inchesToMeters(24.0);
      public static final double robotWidth = Units.inchesToMeters(30.0);

      public static class Gyroscope {
        public static Port serial_port_id = SPI.Port.kMXP;
      }

      public static class Modules {
        public static double steeringMotorToRotationMechConversion = -7.0 / 150.0;
        public static double driveMotorMotorToRotationMechConversion = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static double driveMotorRotationToLinearConversion = Units.inchesToMeters(4.0) * Math.PI;
        public static double maxModuleSpeedMPS = 6;

        public static class FrontLeft {
          private static double xPositionFromCenter = robotLength / 2;
          private static double yPositionFromCenter = robotWidth / 2;
          public static Translation2d location = new Translation2d(xPositionFromCenter, yPositionFromCenter);

          public static class SteeringMech {
            public static final double motorToMechConversion = steeringMotorToRotationMechConversion;

            public static class PID {
              public static final double kP = 5;
              public static final double kI = 0;
              public static final double kD = 0;
              public static final double minimumInput = 0;
              public static final double maximumInput = 1;
              public static final double toleranceDegrees = 0.0;
              public static final double tolerance = toleranceDegrees / 360.0;
            }

            public static class MotorConfig {
              public static int deviceId = 10;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class DriveMech {

            public static double rotationToLinearConversion = driveMotorMotorToRotationMechConversion;

            public static class MotorConfig {
              public static int deviceId = 20;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class AngleSensor {

            public static class SensorConfig {
              public static int deviceId = 30;
              public static String canbus = "CANIVORE";
            }
          }
        }

        public static class FrontRight {
          private static double xPositionFromCenter = robotLength / 2;
          private static double yPositionFromCenter = -robotWidth / 2;
          public static Translation2d location = new Translation2d(xPositionFromCenter, yPositionFromCenter);

          public static class SteeringMech {

            public static final double motorToMechConversion = steeringMotorToRotationMechConversion;

            public static class PID {
              public static final double kP = 5;
              public static final double kI = 0;
              public static final double kD = 0;
              public static final double minimumInput = 0;
              public static final double maximumInput = 1;
              public static final double toleranceDegrees = 0.0;
              public static final double tolerance = toleranceDegrees / 360.0;
            }

            public static class MotorConfig {
              public static int deviceId = 11;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class DriveMech {

            public static double rotationToLinearConversion = driveMotorMotorToRotationMechConversion;

            public static class MotorConfig {
              public static int deviceId = 21;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class AngleSensor {

            public static class SensorConfig {
              public static int deviceId = 31;
              public static String canbus = "CANIVORE";
            }
          }
        }

        public static class RearLeft {
          private static double xPositionFromCenter = -robotLength / 2;
          private static double yPositionFromCenter = robotWidth / 2;
          public static Translation2d location = new Translation2d(xPositionFromCenter, yPositionFromCenter);

          public static class SteeringMech {

            public static final double motorToMechConversion = steeringMotorToRotationMechConversion;

            public static class PID {
              public static final double kP = 5;
              public static final double kI = 0;
              public static final double kD = 0;
              public static final double minimumInput = 0;
              public static final double maximumInput = 1;
              public static final double toleranceDegrees = 0.0;
              public static final double tolerance = toleranceDegrees / 360.0;
            }

            public static class MotorConfig {
              public static int deviceId = 12;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class DriveMech {

            public static double rotationToLinearConversion = driveMotorMotorToRotationMechConversion;

            public static class MotorConfig {
              public static int deviceId = 22;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class AngleSensor {

            public static class SensorConfig {
              public static int deviceId = 32;
              public static String canbus = "CANIVORE";
            }
          }
        }

        public static class RearRight {
          private static double xPositionFromCenter = -robotLength / 2;
          private static double yPositionFromCenter = -robotWidth / 2;
          public static Translation2d location = new Translation2d(xPositionFromCenter, yPositionFromCenter);

          public static class SteeringMech {

            public static final double motorToMechConversion = steeringMotorToRotationMechConversion;

            public static class PID {
              public static final double kP = 5;
              public static final double kI = 0;
              public static final double kD = 0;
              public static final double minimumInput = 0;
              public static final double maximumInput = 1;
              public static final double toleranceDegrees = 0.0;
              public static final double tolerance = toleranceDegrees / 360.0;
            }

            public static class MotorConfig {
              public static int deviceId = 13;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class DriveMech {

            public static double rotationToLinearConversion = driveMotorMotorToRotationMechConversion;

            public static class MotorConfig {
              public static int deviceId = 23;
              public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
              public static boolean isInverted = false;
              public static IdleMode idleMode = IdleMode.kBrake;
              public static double velocityKP = 0;
              public static double velocityKI = 0;
              public static double velocityKD = 0;
              public static double velocityKF = 1.70e-4;
            }
          }

          public static class AngleSensor {

            public static class SensorConfig {
              public static int deviceId = 33;
              public static String canbus = "CANIVORE";
            }
          }
        }
      }
    }

    public static class Arm {

      public static class Pneumatics {
        public static int pneumaticsModuleID = 19;
        public static PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.REVPH;
      }

      public static class Elevator {

        public static class MechConfig {
          public static double rotationToLinearconversion = Units.inchesToMeters(1.76) * Math.PI; // Andymark 25 series
                                                                                                  // 22
          // tooth sprocket
          public static double motorToMechConversion = (1.0 / 16.0); // REV 4:1, 4:1
        }

        public static class MotorConfig {
          public static int deviceId = 16;
          public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
          public static boolean isInverted = false;
          public static IdleMode idleMode = IdleMode.kBrake;
          public static double velocityKP = 0;
          public static double velocityKI = 0;
          public static double velocityKD = 0;
          public static double velocityKF = 1.70e-4;
        }

        public static class Sensor {
          public static int positionSensorID = 36;
          public static RangingMode mode = RangingMode.Short;
          public static int sampleTime = 24;
        }

        public static class Tilt {
          public static int solenoid1Channel = 1;
          public static int solenoid2Channel = 11;
        }

        public static class Control {
          public static double minPosition = Units.inchesToMeters(0.0);
          public static double maxPosition = Units.inchesToMeters(40.0);
          public static double maxManualSpeedMPS = 0.79;

          public static class PID {
            public static final double kP = 10;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double minimumInput = Double.NaN;
            public static final double maximumInput = Double.NaN;
            public static final double tolerance = 0.02;
          }
        }

      }

      public static class ElbowConstants {

        public static class MechConfig {
          public static double motorToMechConversion = (1.0 / 60.0) * (18.0 / 38.0); // REV 5:1, 4:1, and 3:1 Motor Gear
          // Ratio, //Sprocket Ratio 18 T:38T
        }

        public static class MotorConfig {
          public static int deviceId = 15;
          public static Motor.REVMotor revMotor = Motor.REVMotor.NEO;
          public static boolean isInverted = true;
          public static IdleMode idleMode = IdleMode.kBrake;
          public static double velocityKP = 0;
          public static double velocityKI = 0;
          public static double velocityKD = 0;
          public static double velocityKF = 1.70e-4;
        }

        public static class Sensor {
          public static int dioChannel = 0;
          // public static double offsetDegrees = 210.0; //practicebot
          public static double offsetDegrees = 317.0; // compbot
        }

        public static class Control {

          public static double minAngleDegrees = 3.5;
          public static double maxAngleDegrees = 120;
          public static double maxManualSpeedRotationsPerSecond = 0.125;

          public static class PID {
            public static final double kP = 7.5;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double minimumInput = 0;
            public static final double maximumInput = 1;
            public static final double toleranceDegrees = 1;
            public static final double tolerance = toleranceDegrees / 360.0;
          }

        }

      }

      public static class Claw {

        public static class LimitSwitch {
          public static double stopPositionMeters = 0.005;
          public static double minTruePositionMeters = 0.00;
          public static double maxTruePositionMeters = 0.05;

          public static class Sensor {
            public static int positionSensorID = 34;
            public static RangingMode mode = RangingMode.Short;
            public static int sampleTime = 24;
          }
        }

        public static class Control {
          public static double maxManualSpeedRPS = 15.27;
          public static int doubleSolenoidChannel1 = 0;
          public static int doubleSolenoidChannel2 = 15;
        }

        public static class LeftClaw {

          public static class MechConfig {
            public static double motorToMechConversion = 1.0 / 12.0; // Versa Planetary 12:1
          }

          public static class MotorConfig {
            public static int deviceId = 14;
            public static Motor.REVMotor revMotor = Motor.REVMotor.NEO550;
            public static boolean isInverted = false;
            public static IdleMode idleMode = IdleMode.kBrake;
            public static double velocityKP = 0;
            public static double velocityKI = 0;
            public static double velocityKD = 0;
            public static double velocityKF = 1.70e-4;
          }
        }

        public static class RightClaw {

          public static class MechConfig {
            public static double motorToMechConversion = 1.0 / 12.0; // Versa Planetary 12:1
          }

          public static class MotorConfig {
            public static int deviceId = 24;
            public static Motor.REVMotor revMotor = Motor.REVMotor.NEO550;
            public static boolean isInverted = false;
            public static IdleMode idleMode = IdleMode.kBrake;
            public static double velocityKP = 0;
            public static double velocityKI = 0;
            public static double velocityKD = 0;
            public static double velocityKF = 1.70e-4;
          }
        }

      }
    }
  }

  public static class Auto {

    public static class Drive {

      public static class PortalPositions {
        public static Pose2d leftPortal = new Pose2d(15.50, 7.333, new Rotation2d());
        public static Pose2d rightPortal = new Pose2d(15.50, 6.00, new Rotation2d());
        public static Pose2d dropPortal = new Pose2d(14.54, 7.44, Rotation2d.fromDegrees(90));
      }

      public static class ScoringPositions {
        private static Pose2d position1 = new Pose2d(1.95, 4.95, Rotation2d.fromDegrees(180));
        private static Pose2d position2 = new Pose2d(1.95, 4.40, Rotation2d.fromDegrees(180));
        private static Pose2d position3 = new Pose2d(1.95, 3.85, Rotation2d.fromDegrees(180));
        private static Pose2d position4 = new Pose2d(1.95, 3.30, Rotation2d.fromDegrees(180));
        private static Pose2d position5 = new Pose2d(1.95, 2.75, Rotation2d.fromDegrees(180));
        private static Pose2d position6 = new Pose2d(1.95, 2.20, Rotation2d.fromDegrees(180));
        private static Pose2d position7 = new Pose2d(1.95, 1.60, Rotation2d.fromDegrees(180));
        private static Pose2d position8 = new Pose2d(1.95, 1.05, Rotation2d.fromDegrees(180));
        private static Pose2d position9 = new Pose2d(1.95, 0.50, Rotation2d.fromDegrees(180));
        private static Pose2d[] positionsArray = {
            position1,
            position2,
            position3,
            position4,
            position5,
            position6,
            position7,
            position8,
            position9
        };
        public static List<Pose2d> positionsList = Arrays.asList(positionsArray);
      }
    }

    public static class Arm {

      public static class Alignment {
        public static class High {
          public static class Cone {
            public static final double elevatorPositionMeters0 = 0.25;
            public static final double elbowPositionDegrees0 = 40.0;
            public static final ElevatorTilt.State elevatorTiltState0 = ElevatorTilt.State.NONE;
            public static final double elevatorPositionMeters1 = 0.914;
            public static final double elbowPositionDegrees1 = 16.6;
            public static final ElevatorTilt.State elevatorTiltState1 = ElevatorTilt.State.SIX;
            public static final double waitForElevatorToTilt = 0.5;
          }

          public static class Cube {
            public static final double elevatorPositionMeters0 = 0.914;
            public static final double elbowPositionDegrees0 = 35.0;
            public static final ElevatorTilt.State elevatorTiltState0 = ElevatorTilt.State.NONE;
            public static final double elevatorPositionMeters1 = 0.914;
            public static final double elbowPositionDegrees1 = 35.0;
            public static final ElevatorTilt.State elevatorTiltState1 = ElevatorTilt.State.SIX;
            public static final double waitForElevatorToTilt = 0.5;
          }
        }

        public static class Middle {
          public static class Cone {
            public static final double elevatorPositionMeters = 0.495;
            public static final double elbowPositionDegrees = 29.0;
            public static final ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.SIX;
            public static final double waitForElevatorToTilt = 0.5;
          }

          public static class Cube {
            public static final double elevatorPositionMeters = 0.495;
            public static final double elbowPositionDegrees = 34.0;
            public static final ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.SIX;
            public static final double waitForElevatorToTilt = 0.5;
          }

        }

        public static class Low {
          public static class Cone {
            public static final double elevatorPositionMeters = 0.0;
            public static final double elbowPositionDegrees = 115.0;
            public static final ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.TWO;
            public static final double waitForElevatorToTilt = 0.5;
          }

          public static class Cube {
            public static final double elevatorPositionMeters = 0.0;
            public static final double elbowPositionDegrees = 115.0;
            public static final ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.TWO;
            public static final double waitForElevatorToTilt = 0.5;
          }
        }
      }

      public static class Placement {

        public static class High {
          public static class Cone {
            public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.EIGHT;
            public static double waitForElevatorToTilt = 0.5;
          }

          public static class ConePoof {
            public static double waitForPoof = 0.5;
            public static double grabberSpeedRPM = -916.0;
          }

          public static class Cube {
            public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.SIX;
            public static double waitForElevatorToTilt = 0.25;
            public static double grabberSpeedCubeRPM = -916.0;
          }
        }

        public static class Middle {
          public static class Cone {
            public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.SIX;
            public static double waitForElevatorToTilt = 0.5;
          }

          public static class Cube {
            public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.SIX;
            public static double waitForElevatorToTilt = 0.25;
            public static double grabberSpeedCubeRPM = -916.0;
          }
        }

        public static class Low {
          public static class Cone {
            public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.TWO;
            public static double waitForElevatorToTilt = 0.25;
          }

          public static class Cube {
            public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.TWO;
            public static double waitForElevatorToTilt = 0.25;
            public static double grabberSpeedCubeRPM = -916.0;
          }
        }
      }

      public static class Reset {

        public static class High {

          public static double elevatorPosition = 0.0;
          public static double elbowPosition = 30.0;
          public static double grabberSpeed = 0.0;
          public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.NONE;
        }

        public static class Middle {

          public static double elevatorPosition = 0.0;
          public static double elbowPosition = 30.0;
          public static double grabberSpeed = 0.0;
          public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.NONE;
        }

        public static class Low {
          public static double elevatorPosition = 0.0;
          public static double elbowPosition = 30.0;
          public static double grabberSpeed = 0.0;
          public static ElevatorTilt.State elevatorTiltState = ElevatorTilt.State.NONE;
        }

      }

      public static class Pickup {
        public static double grabberSpeedRPM = 916.0;

        public static class Floor {
          public static class Cone {
            public static final double elevatorPositionMeters = 0.0;
            public static final double elbowPositionDegrees = 112;
            public static final ElevatorTilt.State elevatorTiltState = State.TWO;
            public static final double grabberSpeedRPM = 916.0;
          }

          public static class Cube {
            public static final double elevatorPositionMeters = 0.0;
            public static final double elbowPositionDegrees = 106.5;
            public static final ElevatorTilt.State elevatorTiltState = State.TWO;
            public static final double grabberSpeedRPM = 250.0;
          }
        }

        public static class Drop {
          public static class Cone {
            public static final double elevatorPositionMeters = 0.0;
            public static final double elbowPositionDegrees = 54.97;
            public static final ElevatorTilt.State elevatorTiltState = State.NONE;
            public static final double grabberSpeedRPM = 916.0;
          }

          public static class Cube {
            public static final double elevatorPositionMeters = 0.0;
            public static final double elbowPositionDegrees = 46.84;
            public static final ElevatorTilt.State elevatorTiltState = State.NONE;
            public static final double grabberSpeedRPM = 500.0;
          }
        }

        public static class Sliding {
          public static class Cone {
            public static final double elevatorPositionMeters = 0.650;
            public static final double elbowPositionDegrees = 90.0;
            public static final ElevatorTilt.State elevatorTiltState = State.NONE;
            public static final double grabberSpeedRPM = 916.0;
          }

          public static class Cube {
            public static final double elevatorPositionMeters = 0.680;
            public static final double elbowPositionDegrees = 87.0;
            public static final ElevatorTilt.State elevatorTiltState = State.NONE;
            public static final double grabberSpeedRPM = 500.0;
          }
        }
      }

      public static class Carry {
        public static class Cone {
          public static final double elevatorPositionMeters = 0.0;
          public static final double elbowPositionDegrees = 23.0;
          public static final double grabberSpeedRPM = 0.0;
          public static final ElevatorTilt.State elevatorTiltState = State.NONE;
        }

        public static class Cube {
          public static final double elevatorPositionMeters = 0.0;
          public static final double elbowPositionDegrees = 17.0;
          public static final double grabberSpeedRPM = 0.0;
          public static final ElevatorTilt.State elevatorTiltState = State.NONE;
        }

      }

      public static class LEDLighting {
        public static Color8Bit Cone = new Color8Bit(0xFF, 0XD7, 0X00);
        public static Color8Bit Cube = new Color8Bit(0x3C, 0X09, 0X49);
      }
    }
  }

  public static class AutoRoutines {

    public static class Element3 {
      public static class position2 {
        public static String pathName = "3ElementPosition2";
        public static PIDConstants translationConstants = new PIDConstants(5, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(5, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(2, 2);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position8 {
        public static String pathName = "3ElementPosition8";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(20, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(2, 2);
        public static PathConstraints[] remainingPathConstraints = {};
      }
    }

    public static class Element2 {
      public static class position2 {
        public static String pathName = "ConsumeMassQuantities1";
        public static PIDConstants translationConstants = new PIDConstants(15.000, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10.000, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(2, 2);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position8 {
        public static String pathName = "ConsumeMassQuantities2";
        public static PIDConstants translationConstants = new PIDConstants(5, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(0.5, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(2, 2);
        public static PathConstraints[] remainingPathConstraints = {};
      }
    }

    public static class Element1 {

      public static class position4 {
        public static String pathName = "ChargeUp1";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(4, 3);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position5Balance {
        public static String pathName = "ChargeUpMid";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(4, 3);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position5 {
        public static String pathName = "WeComeFromFrance";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(1.25, 2);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position6 {
        public static String pathName = "ChargeUp2";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(4, 3);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position5Cone {
        public static String pathName = "IAmWithCone";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(4, 3);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position5Cone1 {
        public static String pathName = "IAmWithCone1";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(4, 3);
        public static PathConstraints[] remainingPathConstraints = {};
      }

      public static class position5Cone2 {
        public static String pathName = "IAmWithCone2";
        public static PIDConstants translationConstants = new PIDConstants(10, 0, 0);
        public static PIDConstants rotationConstants = new PIDConstants(10, 0, 0);
        public static PathConstraints firstPathConstraint = new PathConstraints(4, 3);
        public static PathConstraints[] remainingPathConstraints = {};
      }

    }

  }
}
