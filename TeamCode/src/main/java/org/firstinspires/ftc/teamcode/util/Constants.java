package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    @Config
    public static class RobotConstants{
        public enum Alliance {BlueAlliance, RedAlliance}
        public static Alliance selectedAlliance = Alliance.BlueAlliance;




        public enum DriveMode {RobotCentric, FieldCentric}
        public static DriveMode selectedDriveMode = DriveMode.FieldCentric;




        public static String frontLeftMotorName = "front_left_motor";
        public static String backLeftMotorName = "back_left_motor";
        public static String frontRightMotorName = "front_right_motor";
        public static String backRightMotorName = "back_right_motor";

        public static boolean frontLeftMotorInverted = true;
        public static boolean backLeftMotorInverted = true;
        public static boolean frontRightMotorInverted = false;
        public static boolean backRightMotorInverted = false;

        public static boolean floatModeEnabled = false;

        public static double minimumDriveSpeed=0.315;
        public static double driveCubicTerm=0.5, driveLinearTerm=0.4;
        public static double headingKp=2.2, headingKd=0.0, targetingOffset=3.0;




        public static String imuName = "imu";
        public static String pinpointDriverName = "pinpoint";

        public static double xPodOffset=0.0, yPodOffset=0.0;

        public static boolean xPodInverted = false;
        public static boolean yPodInverted = false;
    }





    @Config
    public static class FieldConstants{
        public static double fullFieldSize=144.0;
        public static double blueAllianceGoalXPosition=130.0, blueAllianceGoalYPosition=135.0;
        public static double redAllianceGoalXPosition=14.0, redAllianceGoalYPosition=135.0;




        public enum StartingPosition {
            blueAllianceFarZoneStartPosition(88.0, 12.0, 0.0),
            blueAllianceCloseZoneStartPosition(105.0, 138.0, 0.0),
            redAllianceFarZoneStartPosition(58.0, 12.0, 0.0),
            redAllianceCloseZoneStartPosition(41.0, 138.0, 0.0);

            public final double x;
            public final double y;
            public final double headingDeg;

            StartingPosition(double x, double y, double headingDeg) {
                this.x = x;
                this.y = y;
                this.headingDeg = headingDeg;
            }
        }
        public static StartingPosition selectedStartingPosition = StartingPosition.blueAllianceCloseZoneStartPosition;
    }
}
