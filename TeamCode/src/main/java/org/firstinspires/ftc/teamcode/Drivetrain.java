package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//import org.firstinspires.ftc.teamcode.roadrunner.KiwiKinematics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Drivetrain {
    DcMotorEx[] motors;
    BNO055IMU imu;
    float initialAngularOffset;
    double maxPower;
    private final double TICKS_PER_REV = 1; //TODO
    private final double WHEEL_DIA = 1; //TODO
    private final double BASE_RADIUS = 1;



    public enum DriveMode {
        LOCAL,
        FIELD,
        BEYBLADE
    }


    public Drivetrain(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, BNO055IMU imu, float initialAngularOffset, double maxPower) {
        motors = new DcMotorEx[]{motor1, motor2, motor3};
        this.imu = imu;
        this.initialAngularOffset = initialAngularOffset;
        this.maxPower = maxPower;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Map<String, Object> update(double drive, double strafe, double twist) {
        Map<String, Object> telemetry = new HashMap<>();
        float angle = initialAngularOffset + -imu.getAngularOrientation().firstAngle; // TODO: Refactor to use pose estimate for angle consistency

        double[] motorPowers = calcMotorPowers(angle, drive, strafe, twist);

        for (int i = 0; i < motors.length; i++) {
            telemetry.put("Motor " + i, motorPowers[i]);
            motors[i].setPower(motorPowers[i]);
        }

        telemetry.put("Angle", angle);
        return telemetry;
    }

    public double[] calcMotorPowers(float angle, double drive, double strafe, double twist) {
        double adj_strafe = strafe, adj_drive = drive;
        if (Constants.driveMode != DriveMode.LOCAL) {
            adj_strafe = strafe * Math.cos(angle) - drive * Math.sin(angle);
            adj_drive = strafe * Math.sin(angle) + drive * Math.cos(angle);
        }

        double motorPower1 = (-adj_strafe + twist) * maxPower;
        double motorPower2 = (adj_strafe/2 + 0.866*adj_drive + twist) * maxPower;
        double motorPower3 = (adj_strafe/2 - 0.866*adj_drive + twist) * maxPower;

        if (Constants.driveMode == DriveMode.BEYBLADE) {
             motorPower1 = (-adj_strafe) * 0.5 + 0.5;
             motorPower2 = (adj_strafe/2 + adj_drive) * 0.5 + 0.5;
             motorPower3 = (adj_strafe/2 - adj_drive) * 0.5 + 0.5;
        }


        return normalizeMotorPowers(new double[]{motorPower1, motorPower2, motorPower3});

    }

    private double[] normalizeMotorPowers(double[] values) {
        double max = 1;
        double[] normalizedValues = new double[3];
        for (double value : values) {
            if (Math.max(value, max) == value) {
                max = value;
            }
        }
        for (int i = 0; i < values.length; i++) {
            normalizedValues[i] = values[i]/max;
        }
        return normalizedValues;

    }

    public List<Double> getWheelPositions() {
        ArrayList<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(toInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    private double toInches(int encCount) {
        return (encCount/TICKS_PER_REV) * 2 * Math.PI * WHEEL_DIA;
    }

    private void setMotorPowers(double[] powers) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    private void setMotorPowers(List<Double> powers) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers.get(i));
        }
    }






}