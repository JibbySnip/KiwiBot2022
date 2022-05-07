package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Constants {
    public static int POSITION_TOLERANCE = 1;
    public static PIDFCoefficients armCoeffs = new PIDFCoefficients(0.01,0.00007,0.001,0);
    public static double armSetAngle = 0.0;
    public static double dumpSetOffset = 0.0;
    public static double CAROUSEL_SPEED = 0.6;
    public static Drivetrain.DriveMode driveMode = Drivetrain.DriveMode.FIELD;
    public static boolean DEBUG_ARM = false;
    public static boolean DEBUG_DUMP = false;
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
}
