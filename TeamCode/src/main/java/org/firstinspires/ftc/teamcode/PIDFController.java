package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.sun.tools.javac.util.ListBuffer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Queue;

public class PIDFController {
    private PIDFCoefficients coeffs;
    private final int iZone;
    private double setpoint = 0;
    private double error;
    private final ArrayList<Double> accumulatedError;
    private double tolerance;

    public PIDFController(PIDFCoefficients coeffs, int iZone, double tolerance) {
        this.coeffs = coeffs;
        this.iZone = iZone;
        accumulatedError = new ArrayList<>();
        this.tolerance = tolerance;
    }

    public void updateCoeffs(PIDFCoefficients coeffs) {
        this.coeffs = coeffs;
    }

    public double calculate(double currValue) {
        error = setpoint - currValue;
        progressErrorQueue(error);
        if (Math.abs(error) < tolerance) return 0;
        return coeffs.p * error + coeffs.i * sumError() + coeffs.d * (accumulatedError.get(1) - accumulatedError.get(0)) + coeffs.f;
    }

    public double getError() {
        return error;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void progressErrorQueue(double entry) {
        accumulatedError.add(0,entry);
        while (accumulatedError.size() > iZone) {
            accumulatedError.remove(iZone); // TODO maybe revert
        }
    }

    public double sumError() {
        double sum = 0;
        for (double num : accumulatedError) {
            sum += num;
        }
        return sum;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

}
