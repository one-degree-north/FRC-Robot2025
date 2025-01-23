// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class RegularConstants {

    public static final class MiscConstants {
        public static final int controllerID = 0;
        public static final int zeroSwitchID = 0;
    }

    public static final class CoralConstants {
        public static int leftRollerID = 0;
        public static int rightRollerID = 0;
        public static int wristRollerID = 0;

        public static double wristkP = 0;
        public static double wristkI = 0;
        public static double wristkD = 0;
        public static double wristkS = 0;
        public static double wristkV = 0;
        public static double wristkA = 0;

        public static double rollerIntakeVoltage = 0;
        public static double rollerOuttakeVoltage = 0;
        public static double wristIntakePos = 0;
        public static double wristReefPos = 0;
        public static double wristDockedPos = 0;

        public static double wristMechanismRatio = 1/12;
        public static double wristAllowedError = 0.05;

        public static double wristMMAcceleration = 0;
        public static double wristMMCruiseVelocity = 0;
        public static double wristMMJerk = 0;
    }

    public static final class AlgaeConstants{
        public static int leftFlywheelID = 0;
        public static int rightFlywheelID = 0;
        public static int innerRollersID = 0;
        public static int leftPivotID = 0;
        public static int rightPivotID = 0;

        public static double flywheelkP = 0;
        public static double flywheelkI = 0;
        public static double flywheelkD = 0;
        public static double flywheelkS = 0;
        public static double flywheelkV = 0;
        public static double flywheelkA = 0;

        public static double flywheelMMAcceleration = 0;
        public static double flywheelMMCruiseVelocity = 0;
        public static double flywheelMMJerk = 0;

        public static double pivotkP = 0;
        public static double pivotkI = 0;
        public static double pivotkD = 0;
        public static double pivotkS = 0;
        public static double pivotkV = 0;
        public static double pivotkA = 0;
        
        public static double pivotMMAcceleration = 0;
        public static double pivotMMCruiseVelocity = 0;
        public static double pivotMMJerk = 0;

        public static double flywheelIntakeVelocity = 0;
        public static double flywheelShootVelocity = 0;
        public static double innerRollersVoltage = 0;
        public static double pivotIntakePos = 0;
        public static double pivotProcessorPos = 0;
        public static double pivotLvl2Pos = 0;
        public static double pivotShootingPos = 0;

        public static double flywheelMechanismRatio = 1/12;
        public static double pivotAllowedError = 0.05;
    }

    public static final class ElevatorConstants {
        public static int leftElevatorID = 0;
        public static int rightElevatorID = 0;
        public static int elevatorEncoderID = 0;
        public static int magneticLimitSwitchID = 0;

        public static double elevatorUpVoltage = 0;
        public static double elevatorDockedPos = 0;
        public static double elevatorL1Pos = 0;
        public static double elevatorL2Pos = 0;
        public static double elevatorL3Pos = 0;
        public static double elevatorL4Pos = 0;


        public static double ElevatorkP = 0;
        public static double ElevatorkI = 0;
        public static double ElevatorkD = 0;
        public static double ElevatorkS = 0;
        public static double ElevatorkV = 0;
        public static double ElevatorkA = 0;
        public static double ElevatorkG = 0;

        public static double ElevatorMechanismRatio = 1/12;

        public static double ElevatorMMAcceleration = 0;
        public static double ElevatorMMCruiseVelocity = 0;
        public static double ElevatorMMJerk = 0;

        public static double wristAllowedError = 0;


    }


}
