package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

public class Arm {

    public enum ArmState {
        PICKUP, DEPOSIT_LOW, DEPOSIT_MID, DEPOSIT_HIGH, DEBUG
    }
    public enum DumpState {
        PICKUP, HOLD, DUMP, DEBUG
    }
    DcMotorEx armMotor;
    Servo dumpServo;

    DumpState dumpState;
    ArmState armState;

    public PIDFController armController;

    private final double TICKS_PER_REV = 537.7*2; //TODO: Find the correct number
    private final double DUMP_ANGLE_RANGE = 160; // TODO: Fix to angular range
    private final double FREIGHT_MIN_DIST = 1; // TODO: Find empirically

    private final Map<ArmState, Double> armPoses = new HashMap<>();
    private final Map<DumpState, Double> dumpPoses = new HashMap<>();

    public Arm(DcMotorEx armMotor, Servo dumpServo) {

        this.armMotor = armMotor;
        this.dumpServo = dumpServo;

        armPoses.put(ArmState.PICKUP, 0.0);
        armPoses.put(ArmState.DEPOSIT_LOW, 0.0);
        armPoses.put(ArmState.DEPOSIT_MID, 0.0);
        armPoses.put(ArmState.DEPOSIT_HIGH, 0.0);
        armPoses.put(ArmState.DEBUG, Constants.armSetAngle);

        dumpPoses.put(DumpState.PICKUP, 0.0);
        dumpPoses.put(DumpState.HOLD, 0.0);
        dumpPoses.put(DumpState.DUMP, 0.0);
        dumpPoses.put(DumpState.DEBUG, Constants.dumpSetOffset);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armController = new PIDFController(Constants.armCoeffs, 10, Constants.POSITION_TOLERANCE);
    }

    public Map<String, Object> update(ArmState armState, DumpState dumpState) {
        Map<String, Object> telemetry = new HashMap<>();
//        PIDFCoefficients mCoeffs = Constants.armCoeffs;
//        mCoeffs.f = Constants.armCoeffs.f * Math.cos(Math.toRadians(encoderToAngle(armMotor.getCurrentPosition())) - 50);
        armController.updateCoeffs(Constants.armCoeffs);

        armPoses.put(ArmState.DEBUG, Constants.armSetAngle);
        dumpPoses.put(DumpState.DEBUG, Constants.dumpSetOffset);

        this.armState = Constants.DEBUG_ARM ? ArmState.DEBUG : armState;
        this.dumpState = Constants.DEBUG_DUMP ? DumpState.DEBUG : dumpState;

        double dumpSetpoint = calcDumpPos(this.dumpState, armMotor.getCurrentPosition());

        armController.setSetpoint(armPoses.get(this.armState));
        armMotor.setPower(armController.calculate(encoderToAngle(armMotor.getCurrentPosition())));
        dumpServo.setPosition(dumpSetpoint);

        telemetry.put("armSetEncoderCount", angleToEncoderPos(armPoses.get(armState)));
        telemetry.put("armSetAngle", armPoses.get(armState));
        telemetry.put("armCurrentEncoderCount", armMotor.getCurrentPosition());
        telemetry.put("armCurrentAngle", encoderToAngle(armMotor.getCurrentPosition()));
        telemetry.put("dumpSetpoint", dumpSetpoint);
        telemetry.put("ArmPower", armMotor.getPower());
        telemetry.put("armState", armState);
        telemetry.put("Arm Setpoint", armController.getSetpoint());
        telemetry.put("Arm Error", armController.getError());
        telemetry.put("dumpState", dumpState);

        return telemetry;
    }

    public int angleToEncoderPos(double angle) {
        return (int) ((angle / 360) * TICKS_PER_REV);
    }

    public double encoderToAngle(int ticks) {
        return (ticks/TICKS_PER_REV) * 360;
    }

    public boolean inPosition() {
        return Math.abs(angleToEncoderPos(armPoses.get(armState)) - armMotor.getCurrentPosition()) < 20;
    }

    public double calcDumpPos(DumpState dumpMode, int encoderCount) {
        double dumpOffset = dumpPoses.get(dumpMode);
        double dumpAngleOffset = encoderToAngle(encoderCount);

        // Dump angle is equal to the arm angle, and the offset is added

        return Math.min((dumpOffset + dumpAngleOffset) / DUMP_ANGLE_RANGE, 1);
    }
}
