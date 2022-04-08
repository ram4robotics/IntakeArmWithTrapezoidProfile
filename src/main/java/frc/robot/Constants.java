// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class CAN_IDs {
        public final static int left1_ID = 11;
        public final static int left2_ID = 12;
        public final static int right1_ID = 13;
        public final static int right2_ID = 14;
        public final static int intakeWheels_ID = 21;
        public final static int intakeArm_ID = 22;
        public final static int indexerFront_ID = 23;
        public final static int indexerBack_ID = 24;
        public final static int launcher1_ID = 25;
        public final static int launcher2_ID = 26;
        public final static int climber1_ID = 31;
        public final static int climber2_ID = 32;
    }
    public final static class IntakeArmConstants {
        // The following FeedForward estimates are theoritical values calculated from recalc tool (reca.lc)
        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&comLength=
        // %7B%22s%22%3A10%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A60%2C%22u%22%3A
        // %22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A110%2C%22u%22%3A%22deg%22%7D&
        // iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&
        // ratio=%7B%22magnitude%22%3A30%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=
        // %7B%22s%22%3A-20%2C%22u%22%3A%22deg%22%7D
        // Note: 0.3 power was found to be sufficient to move the Arm up. 0.3 power = 12*0.3 = 3.6Volts
        public final static double kGVolts = 1.61; // Volts
        public final static double kSVolts = 3.6-kGVolts; // armkSVolts = Volts_that_take_to_move_the_arm_from_rest - 
                                                                //               armkSVolts
        public final static double kVVoltSecondPerRad = 0.58; // Volts * sec / radians
        public final static double kAVoltSecondSquaredPerRad = 0.07; // Volts * sec^2 / radians

        // The following Feedback estimates are taken from the WPILIB's ArmbotOffboard example
        // PID values for SparkMaxPIDController
        public final static double kP = 0.1;
        public final static double kI = 0;
        public final static double kD = 0;
        // Min and Max output power allowed by SparkMaxPIDController
        public final static double kMinOutput = -0.5;
        public final static double kMaxOutput = 0.75;

        // Arm Positions 
        private final static double _closedPosArmDegrees = 10; // was 0; Position of the Arm 1x1 tubing on the robot
        private final static double _openedPosArmDegrees = -100; // was -125; Position of the Arm 1x1 tubing on the robot
        public final static double kClosedPosArmRad = Units.degreesToRadians(_closedPosArmDegrees);
        public final static double kOpenedPosArmRad = Units.degreesToRadians(_openedPosArmDegrees);
        private final static double _gearRatio = (4*3*60/24); // 4:1 cartridge + 3:1 cartrdige + 60T:24T sprockets
        // The following two values in terms of Neo motor shaft rotations
        private final static double _closedPosNeoRotations = Units.degreesToRotations(_closedPosArmDegrees * _gearRatio);
        private final static double _openedPosNeoRotations = Units.degreesToRotations(_openedPosArmDegrees * _gearRatio);

        // Arm position limits in units of Neo Motor shaft rotations
        // Add 10% leeway for setting the softlimit
        public final static float kForwardSoftlimit = (float)_closedPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations
        public final static float kReverseSoftLimit = (float)_openedPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations

        // The following are constrains for the TrapezoidalProfile
        // Note that the TrapezoidalProfile takes values in Radians whereas SparkMax's PIDController
        // use number of Shaft Rotations
        // Note: Max velocity the neo+30:1 reduction can give is 14.4 radians / sec. (from AriMB design spreadsheet)
        private final static double _maxVelocityArmDegPerSec = 100; // => 1.2 seconds from closed-to-opened position
        private final static double _secondsToPeakVel = 1; // => 1 second to 0-to-peak velocity
        private final static double _maxAccelArmDegPerSecSquared = _maxVelocityArmDegPerSec / _secondsToPeakVel; 
        public final static double kMaxVelocityRadPerSecond = Units.degreesToRadians(_maxVelocityArmDegPerSec);
        public final static double kMaxAccelerationRadPerSecSquared = Units.degreesToRadians(_maxAccelArmDegPerSecSquared);
        public final static double kInitialPositionRad = 0; // was kClosedPosArmRad
        public final static double kArmDegToNeoRotConversionFactor = Units.degreesToRotations(_gearRatio);

        // The following fixed speeds are applicable only if not using the FeedForward or Feedback controllers
        public final static double armMotorOpeningSpeed = -0.2;
        public final static double armMotorClosingSpeed = 0.3;
    }
    public final class OIConstants {
        public final static int xbox1_port = 0;
        public final static int xbox2_port = 1;
    }
}
