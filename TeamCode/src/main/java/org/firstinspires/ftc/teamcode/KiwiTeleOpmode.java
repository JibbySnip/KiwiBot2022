package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robot.robot;

@TeleOp(name="KiwiOp")
public class KiwiTeleOpmode extends OpMode {
    Robot mRobot = robot;
    Arm.ArmState armState = Arm.ArmState.PICKUP;
    Arm.DumpState dumpState = Arm.DumpState.HOLD;
    Spinner.SpinState spinState = Spinner.SpinState.OFF;
    private boolean rStickButtonPressed = false;

    @Override
    public void init() {
       mRobot.init(telemetry, hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.initImu(parameters);
    }

    @Override
    public void loop() {

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double twist = -gamepad1.right_stick_x;

        // Arm states

        if (gamepad1.x) {
            armState = Arm.ArmState.PICKUP;
            if (dumpState != Arm.DumpState.HOLD) { // If we shouldn't be stowed for warehouse
                dumpState = Arm.DumpState.PICKUP; // Go into pickup
            }
        } else if (gamepad1.y) {
            armState = Arm.ArmState.DEPOSIT_HIGH;
            dumpState = Arm.DumpState.HOLD;
        } else if (gamepad1.a) {
            armState = Arm.ArmState.DEPOSIT_LOW;
            dumpState = Arm.DumpState.HOLD;
        } else if (gamepad1.b){
            armState = Arm.ArmState.DEPOSIT_MID;
            dumpState = Arm.DumpState.HOLD;
        }

        // Dump states

        if (gamepad1.left_bumper) { // Controls pickup/hold
            if (dumpState != Arm.DumpState.PICKUP) {
                dumpState = Arm.DumpState.PICKUP;
                armState = Arm.ArmState.PICKUP;
            } else {
                dumpState = Arm.DumpState.HOLD;
            }
        }

        if (gamepad1.right_bumper) { // Controls dump/hold
            if (armState != Arm.ArmState.PICKUP) {
                if (dumpState != Arm.DumpState.DUMP) {
                    dumpState = Arm.DumpState.DUMP;
                } else { // Stows arm for travel after deposit
                    armState = Arm.ArmState.PICKUP;
                    dumpState = Arm.DumpState.HOLD;
                }
            }
        }

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            spinState = Spinner.SpinState.ON;
        } else {
            spinState = Spinner.SpinState.OFF;
        }

        if (!rStickButtonPressed && gamepad1.right_stick_button) {
            robot.toggleBeyblade();
        }
        rStickButtonPressed = gamepad1.right_stick_button;

        if (Constants.DEBUG_ARM) {
            armState = Arm.ArmState.DEBUG;
        }
        if (Constants.DEBUG_DUMP) {
            dumpState = Arm.DumpState.DEBUG;
        }

        mRobot.update(drive, strafe, twist, armState, dumpState, spinState);
    }
}
