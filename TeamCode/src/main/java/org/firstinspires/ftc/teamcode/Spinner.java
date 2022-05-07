package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;
import java.util.Map;

public class Spinner {
    DcMotorEx spinner;
    enum SpinState {
        ON, OFF
    }
    private SpinState spinState = SpinState.OFF;

    public Spinner(DcMotorEx spinner) {
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.spinner = spinner;
    }

    public Map<String, Object> update(SpinState spinState) {
        Map<String, Object> telemetry = new HashMap<>();
        this.spinState = spinState;
        if (spinState == SpinState.ON) {
            spinner.setPower(Constants.CAROUSEL_SPEED);
        } else {
            spinner.setPower(0);
        }
        telemetry.put("Spin State", spinState);
        telemetry.put("RPM", spinner.getVelocity(AngleUnit.DEGREES) / 360);
        return telemetry;
    }

}
