package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@TeleOp(name="Hyperion Bots Regionals TeleOp", group="TeleOp")
public class RegionalsTeleOp extends OpMode {
    private DcMotor front_left_motor, back_left_motor, front_right_motor, back_right_motor;





    IMU imu;
    GoBildaPinpointDriver pinpointDriver;





    private double currentHeading=0.0, targetHeading=0.0, previousHeadingError=0.0;
    private double currentXPosition=0.0, currentYPosition=0.0, distanceToGoal=0.0, angleToGoal=0.0;
    private static final List<CalibrationPoints> calibrationPoints = new ArrayList<>();
    public static double dynamicTargetFlyWheelVelocity=0.0, dynamicPitcherServoPosition=0.0;





    @Override
    public void init(){
        front_left_motor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.frontLeftMotorName);
        back_left_motor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.backLeftMotorName);
        front_right_motor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.frontRightMotorName);
        back_right_motor = hardwareMap.get(DcMotor.class, Constants.RobotConstants.backRightMotorName);

        front_left_motor.setDirection(Constants.RobotConstants.frontLeftMotorInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        back_left_motor.setDirection(Constants.RobotConstants.backLeftMotorInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(Constants.RobotConstants.frontRightMotorInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(Constants.RobotConstants.backRightMotorInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        front_left_motor.setZeroPowerBehavior(Constants.RobotConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(Constants.RobotConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(Constants.RobotConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(Constants.RobotConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);



        imu = hardwareMap.get(IMU.class, Constants.RobotConstants.imuName);
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();


        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, Constants.RobotConstants.pinpointDriverName);
        pinpointDriver.setOffsets(Constants.RobotConstants.xPodOffset, Constants.RobotConstants.yPodOffset, DistanceUnit.INCH);
        pinpointDriver.setEncoderDirections(Constants.RobotConstants.xPodInverted ?
                        GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Constants.RobotConstants.yPodInverted ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();



        initializeCalibrationPoints();
    }





    @Override
    public void init_loop(){
        telemetry.addData("Alliance: ", Constants.RobotConstants.selectedAlliance);
        telemetry.addData("Selected Start Position: ", Constants.FieldConstants.selectedStartingPosition);
        telemetry.update();
    }





    @Override
    public void loop(){
        //----------Calling Functions----------//
        RobotDrive();
        TelemetryLogging();
        OdometryTracking();
        CalculateShooterParameters();
        BackgroundOperations();

        telemetry.update();
    }





    private void BackgroundOperations(){
        pinpointDriver.update();
    }





    private void RobotDrive(){
        targetHeading = Math.toRadians(angleToGoal);
        double headingCorrection = PIDController();

        double adjustedDrivingSpeed = Constants.RobotConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + Constants.RobotConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, Constants.RobotConstants.minimumDriveSpeed);

        double y = -gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x = gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = gamepad1.right_stick_x * adjustedDrivingSpeed;

        double headingOffset = Constants.RobotConstants.selectedAlliance == Constants.RobotConstants.Alliance.RedAlliance ? Math.toRadians(-90) : Math.toRadians(90);
        double rotatedX = x * Math.cos(-currentHeading+headingOffset) - y * Math.sin(-currentHeading+headingOffset);
        double rotatedY = x * Math.sin(-currentHeading+headingOffset) + y * Math.cos(-currentHeading+headingOffset);

        double frontLeftMotorPower = (Constants.RobotConstants.selectedDriveMode == Constants.RobotConstants.DriveMode.RobotCentric ?
                (y + x + rx) : (rotatedY + rotatedX + rx));

        double backLeftMotorPower = (Constants.RobotConstants.selectedDriveMode == Constants.RobotConstants.DriveMode.RobotCentric ?
                (y - x + rx) : (rotatedY - rotatedX + rx));

        double frontRightMotorPower = (Constants.RobotConstants.selectedDriveMode == Constants.RobotConstants.DriveMode.RobotCentric ?
                (y - x - rx) : (rotatedY - rotatedX - rx));

        double backRightMotorPower = (Constants.RobotConstants.selectedDriveMode == Constants.RobotConstants.DriveMode.RobotCentric ?
                (y + x - rx) : (rotatedY + rotatedX - rx));

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftMotorPower), Math.abs(backLeftMotorPower)),
                Math.max(Math.abs(frontRightMotorPower), Math.abs(backRightMotorPower))
        );

        if (maxPower > 1.0) {
            frontLeftMotorPower /= maxPower;
            backLeftMotorPower /= maxPower;
            frontRightMotorPower /= maxPower;
            backRightMotorPower /= maxPower;
        }

        front_left_motor.setPower(frontLeftMotorPower);
        back_left_motor.setPower(backLeftMotorPower);
        front_right_motor.setPower(frontRightMotorPower);
        back_right_motor.setPower(backRightMotorPower);
    }





    private void OdometryTracking(){
        currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Pose2D currentRobotPosition = pinpointDriver.getPosition();

        currentXPosition = Math.max(0, Math.min(Constants.FieldConstants.fullFieldSize, Constants.FieldConstants.selectedStartingPosition.x + currentRobotPosition.getY(DistanceUnit.INCH)));
        currentYPosition = Math.max(0, Math.min(Constants.FieldConstants.fullFieldSize, Constants.FieldConstants.selectedStartingPosition.y + currentRobotPosition.getX(DistanceUnit.INCH)));

        double blueDx = Constants.FieldConstants.blueAllianceGoalXPosition - currentXPosition;
        double blueDy = Constants.FieldConstants.blueAllianceGoalYPosition - currentYPosition;

        double redDx = Constants.FieldConstants.redAllianceGoalXPosition - currentXPosition;
        double redDy = Constants.FieldConstants.redAllianceGoalYPosition - currentYPosition;

        distanceToGoal = (Constants.RobotConstants.selectedAlliance == Constants.RobotConstants.Alliance.BlueAlliance ?
                Math.hypot(blueDx, blueDy) : Math.hypot(redDx, redDy));

        //----------Calculate Angle To Goal----------//
        angleToGoal = (Constants.RobotConstants.selectedAlliance == Constants.RobotConstants.Alliance.BlueAlliance ?
                Math.toDegrees(Math.atan2(blueDx, blueDy)) + Constants.RobotConstants.targetingOffset : Math.toDegrees(Math.atan2(redDx, redDy)) + Constants.RobotConstants.targetingOffset);
    }
    private static class CalibrationPoints{
        public double distanceToGoal;
        public double flyWheelVelocity;
        public double servoPosition;

        public CalibrationPoints(double distanceToGoal, double flyWheelVelocity, double servoPosition){
            this.distanceToGoal = distanceToGoal;
            this.flyWheelVelocity = flyWheelVelocity;
            this.servoPosition = servoPosition;
        }
    }
    private void initializeCalibrationPoints(){
        calibrationPoints.clear();

        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));
        calibrationPoints.add(new CalibrationPoints(0.0, 0.0, 0.0));

        calibrationPoints.sort(Comparator.comparingDouble(a -> a.distanceToGoal));
    }
    private void CalculateShooterParameters(){
        double distance = distanceToGoal;

        if(calibrationPoints.isEmpty()){
            dynamicTargetFlyWheelVelocity=1000.0;
            dynamicPitcherServoPosition=0.0;
            return;
        }

        if(distance <= calibrationPoints.get(0).distanceToGoal){
            dynamicTargetFlyWheelVelocity = calibrationPoints.get(0).flyWheelVelocity;
            dynamicPitcherServoPosition = calibrationPoints.get(0).servoPosition;
            return;
        }

        if(distance >= calibrationPoints.get(calibrationPoints.size() - 1).distanceToGoal){
            CalibrationPoints last = calibrationPoints.get(calibrationPoints.size() - 1);
            dynamicTargetFlyWheelVelocity = last.flyWheelVelocity;
            dynamicPitcherServoPosition = last.servoPosition;
            return;
        }

        for (int i = 0; i < calibrationPoints.size() - 1; i++) {
            CalibrationPoints p1 = calibrationPoints.get(i);
            CalibrationPoints p2 = calibrationPoints.get(i + 1);

            if (distance >= p1.distanceToGoal && distance <= p2.distanceToGoal) {
                CalibrationPoints p0 = (i > 0) ? calibrationPoints.get(i - 1) : p1;
                CalibrationPoints p3 = (i < calibrationPoints.size() - 2) ? calibrationPoints.get(i + 2) : p2;

                double t = (distance - p1.distanceToGoal) / (p2.distanceToGoal - p1.distanceToGoal);
                double t2 = t * t;
                double t3 = t2 * t;

                double m1_vel = (p2.flyWheelVelocity - p0.flyWheelVelocity) / (p2.distanceToGoal - p0.distanceToGoal);
                double m2_vel = (p3.flyWheelVelocity - p1.flyWheelVelocity) / (p3.distanceToGoal - p1.distanceToGoal);

                dynamicTargetFlyWheelVelocity =
                        (2*t3 - 3*t2 + 1) * p1.flyWheelVelocity +
                                (t3 - 2*t2 + t) * (p2.distanceToGoal - p1.distanceToGoal) * m1_vel +
                                (-2*t3 + 3*t2) * p2.flyWheelVelocity +
                                (t3 - t2) * (p2.distanceToGoal - p1.distanceToGoal) * m2_vel;

                double m1_servo = (p2.servoPosition - p0.servoPosition) / (p2.distanceToGoal - p0.distanceToGoal);
                double m2_servo = (p3.servoPosition - p1.servoPosition) / (p3.distanceToGoal - p1.distanceToGoal);

                dynamicPitcherServoPosition =
                        (2*t3 - 3*t2 + 1) * p1.servoPosition +
                                (t3 - 2*t2 + t) * (p2.distanceToGoal - p1.distanceToGoal) * m1_servo +
                                (-2*t3 + 3*t2) * p2.servoPosition +
                                (t3 - t2) * (p2.distanceToGoal - p1.distanceToGoal) * m2_servo;

                dynamicTargetFlyWheelVelocity = Math.max(0, Math.min(2200, dynamicTargetFlyWheelVelocity));
                dynamicPitcherServoPosition = Math.max(0, Math.min(1.0, dynamicPitcherServoPosition));
                return;
            }
        }
    }





    private void TelemetryLogging(){
        telemetry.addData("Drive Mode: ", Constants.RobotConstants.selectedDriveMode);
        telemetry.addData("Alliance: ", Constants.RobotConstants.selectedAlliance);

        telemetry.addData("Current Heading: ", Math.toDegrees(currentHeading));
        telemetry.addData("X Position: ", currentXPosition);
        telemetry.addData("Y Position: ", currentYPosition);
        telemetry.addData("Distance to goal: ", distanceToGoal);
        telemetry.addData("Angle to goal: ", angleToGoal);
    }





    private double PIDController(){
        double headingError = currentHeading - targetHeading;
        headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

        double derivative = headingError - previousHeadingError;
        previousHeadingError = headingError;

        return (headingError * Constants.RobotConstants.headingKp) +
                (derivative * Constants.RobotConstants.headingKd);
    }
}