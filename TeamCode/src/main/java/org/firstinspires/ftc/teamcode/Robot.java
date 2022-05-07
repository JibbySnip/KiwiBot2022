package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class Robot {
    private HardwareMap hardwareMap;
    public BNO055IMU imu;
    private Drivetrain drivetrain;
    private Arm arm;
//    private Spinner spinner;
//    private RevBlinkinLedDriver leds;
    private final float initialAngle = (float) -(Math.PI)/3;
    private final double maxPower = 1.0;
    private boolean isInit = false;
    private FtcDashboard dashboard;
    private Telemetry telemetry;

    public static Robot robot = new Robot();

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        if (!isInit) {

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            imu = hardwareMap.get(BNO055IMU.class, "imu");


            drivetrain = new Drivetrain(
                    hardwareMap.get(DcMotorEx.class, "strafeDrive"),
                    hardwareMap.get(DcMotorEx.class, "rightDrive"),
                    hardwareMap.get(DcMotorEx.class, "leftDrive"),
                    imu,
                    initialAngle,
                    maxPower

            );

            arm = new Arm(
                    hardwareMap.get(DcMotorEx.class, "armMotor"),
                    hardwareMap.get(Servo.class, "dumpServo")
            );

//            spinner = new Spinner(hardwareMap.get(DcMotorEx.class, "spinnerMotor"));

            isInit = true;
        }
        dashboard = FtcDashboard.getInstance();
    }

    public void initImu(BNO055IMU.Parameters parameters) {
        imu.initialize(parameters);
    }

    public void update(double drive, double strafe, double twist, Arm.ArmState armState, Arm.DumpState dumpState, Spinner.SpinState spinState) {
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.putAll(drivetrain.update(drive, strafe, twist));
        telemetry.putAll(arm.update(armState, dumpState));
//        telemetry.putAll(spinner.update(spinState));
//        telemetry.putAll(updateLEDS(armState));
        dashboard.sendTelemetryPacket(telemetry);
    }

//    public boolean hasFreight() {
//        return arm.hasFreight();
//    }

//    public Map<String, Object> updateLEDS(Arm.ArmState armState) {
//        Map<String, Object> telemetry = new HashMap<>();
//        RevBlinkinLedDriver.BlinkinPattern pattern;
//        if (Constants.driveMode == Drivetrain.DriveMode.BEYBLADE) {
//            pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
//        } else if (armState != Arm.ArmState.PICKUP && armState != Arm.ArmState.DEBUG && arm.inPosition()) {
//            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//        } else if (armState == Arm.ArmState.PICKUP && hasFreight()) {
//            pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
//        } else {
//            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK;
//        }
//
//        leds.setPattern(pattern);
//        telemetry.put("LED Pattern", pattern);
//        return telemetry;
//    }

    public void toggleBeyblade() {
        if (Constants.driveMode == Drivetrain.DriveMode.BEYBLADE) {
            Constants.driveMode = Drivetrain.DriveMode.FIELD;
        } else if (Constants.driveMode == Drivetrain.DriveMode.FIELD) {
            Constants.driveMode = Drivetrain.DriveMode.BEYBLADE;
        }
    }
}
