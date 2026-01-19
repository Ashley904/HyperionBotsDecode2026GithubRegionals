package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@Config
public class Constants {
    @Config
    public static class RobotConstants{
        public enum Alliance {BlueAlliance, RedAlliance}
        public static Alliance selectedAlliance = Alliance.BlueAlliance;




        public enum DriveMode {RobotCentric, FieldCentric}
        public static DriveMode selectedDriveMode = DriveMode.FieldCentric;




        public static String frontLeftMotorName = "FL";
        public static String backLeftMotorName = "BL";
        public static String frontRightMotorName = "FR";
        public static String backRightMotorName = "BR";

        public static boolean frontLeftMotorInverted = false;
        public static boolean backLeftMotorInverted = false;
        public static boolean frontRightMotorInverted = true;
        public static boolean backRightMotorInverted = true;

        public static boolean floatModeEnabled = true;

        public static double minimumDriveSpeed=0.315;
        public static double driveCubicTerm=0.5, driveLinearTerm=0.4;
        public static double headingKp=2.2, headingKd=0.0, targetingOffset=3.0;




        public static String imuName = "imu";
        public static String pinpointDriverName = "pinpoint";

        public static double xPodOffset=12.0, yPodOffset=-19.0;

        public static boolean xPodInverted = false;
        public static boolean yPodInverted = true;
    }





    @Config
    public static class GamePadControls{
        public static GamepadEx gamepad1EX;
        public static GamepadEx gamepad2EX;


        public static GamepadKeys.Button driveModeToggleMapping = GamepadKeys.Button.DPAD_UP;
        public static GamepadKeys.Button allianceSelectionToggleMapping = GamepadKeys.Button.DPAD_LEFT;
        public static GamepadKeys.Button resetHeadingToggleMapping = GamepadKeys.Button.START;
        public static GamepadKeys.Button autoAimToggleMapping = GamepadKeys.Button.RIGHT_BUMPER;
        public static GamepadKeys.Button iterateStartingPositionsToggleMapping = GamepadKeys.Button.LEFT_BUMPER;
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
        public static StartingPosition selectedStartingPosition = StartingPosition.blueAllianceFarZoneStartPosition;
    }
}
