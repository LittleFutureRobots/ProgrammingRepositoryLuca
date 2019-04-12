package org.firstinspires.ftc.teamcode.ProgrammingFTC2019LFR;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class AutonomousConfigurationFile {

    private final String VUFORIA_KEY = "AZnVnoj/////AAABmdXzVSC7bkZik9EURkca9g5GwHTQjL0SB5CABkSEajM1oX/nSOWoXxcxH/watnjKf3WlWcGhyPvV0E8eMNZmTbTgrB/8OJhqAflMV+CjgBtERmweuXjLiPcvEgJNrZD7USn+LK53L0VuSYdi4NwJxy7ypbse7jbXlOmJVgogCXbD4+yjYDbnVmBkkMQMhLgIFQZ0wRApvdxc7R/O/rhsQfWrWWekxjIp4wNeYh5JBsCrCRjdPu1P7QLKAMSOpK5lXqJjmD36TPDxqrQEGfdKxkMe2SJta/3tyzc+v/mFRmNDJjqVMYu69eEy6jh7u/KQA2Uj4pdcIfnZhMWwBO58guP2TPl5HCof4weEEUI6ZF8w";
    public HardwareMap hwmap;
    private TFObjectDetector[] tfods = new TFObjectDetector[3];
    private HardwareMap hardwareMapy;
    private DcMotor frontleft, frontright, backleft, backright;
    private BNO055IMU imu;
    private VuforiaLocalizer[] cams = new VuforiaLocalizer[3];
    private DistanceSensor sensorRange;


    double getDistance(DistanceUnit unit) {
        return 0;
    }

    public void initAll(HardwareMap hardwareMapy) {
        hwmap = hardwareMapy;
        initGyro(BNO055IMU.AngleUnit.DEGREES);
        initRobot();
        initVuforia();
        initTfod(2);
    }

    public void initRobot() {

        sensorRange = hwmap.get(DistanceSensor.class, "sensor1");
        frontleft = hwmap.get(DcMotor.class, "frontleft");
        frontright = hwmap.get(DcMotor.class, "frontright");
        backleft = hwmap.get(DcMotor.class, "backleft");
        backright = hwmap.get(DcMotor.class, "backright");

    }


    public void initVuforia() {
        int cameraMonitorViewId = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmap.appContext.getPackageName());
        VuforiaLocalizer.Parameters extern1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        extern1.vuforiaLicenseKey = VUFORIA_KEY;

        extern1.cameraName = hwmap.get(WebcamName.class, "Webcam 1");

        cams[2] = ClassFactory.getInstance().createVuforia(extern1);

    }

    public void initTfod(int index) {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hwmap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hwmap.appContext.getPackageName());

            if (cams[index] != null) {
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
                tfods[index] = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, cams[index]);
                tfods[index].loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");
            }
        } else {
            System.err.println("Cannot init tfod");
        }
    }

    public void runObjectDetection(int cameraIndex, ObjectDetected action) {
        TFObjectDetector tf = tfods[cameraIndex];
        tf.activate();
        for (int i = 0; i < 3; i++) {
            List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
            while (true) {
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() != 0) break;
                }
                updatedRecognitions = tf.getUpdatedRecognitions();
            }
            boolean found = false;
            for (Recognition recogn : updatedRecognitions) {
                if (recogn.getLabel().equals("Gold Mineral")) {
                    found = true;
                }
            }
            if (found) {
                tf.shutdown();
                switch (i) {
                    case 0:
                        action.onCenter();
                    case 1:
                        action.onLeft();
                    case 2:
                        action.onRight();
                }
                return;
            } else {
                if (i == 0) {
                    turnTo(0.3, 40);
                } else {
                    turnTo(0.3, -40);
                }
            }
        }
    }


    public void initGyro(BNO055IMU.AngleUnit angle) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = angle;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;// see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @SuppressLint("DefaultLocale")
    public double getAngle(Axis axis) {
        switch (axis) {
            case X:
                return Double.parseDouble(String.format("%.00f", imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).firstAngle));
            case Y:
                return Double.parseDouble(String.format("%.00f", imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).secondAngle));
            case Z:
                return Double.parseDouble(String.format("%.00f", imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).thirdAngle));
            default:
                return 0;
        }
    }


    @SuppressWarnings("deprecation")
    public void turnTo(double power, int degrees) {

        double angle = getAngle(Axis.Z);
        while (Math.abs(angle - degrees) > 0) {
            angle = getAngle(Axis.Z);

            if (angle < degrees) {
                getLeftMotorUp().setPower(power);
                getRightMotorUp().setPower(power);
                getLeftMotorDown().setPower(power);
                getRightMotorDown().setPower(power);
            } else if (angle > degrees) {
                getLeftMotorUp().setPower(-power);
                getRightMotorUp().setPower(-power);
                getLeftMotorDown().setPower(-power);
                getRightMotorDown().setPower(-power);
            }
        }


        getLeftMotorUp().setPower(0.01);
        getRightMotorUp().setPower(0.01);
        getLeftMotorDown().setPower(0.01);
        getRightMotorDown().setPower(0.01);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        getLeftMotorUp().setPower(0);
        getRightMotorUp().setPower(0);
        getLeftMotorDown().setPower(0);
        getRightMotorDown().setPower(0);
    }

    public DcMotor getLeftMotorUp() {
        return frontleft;
    }

    public DcMotor getRightMotorUp() {
        return frontright;
    }

    public DcMotor getLeftMotorDown() {
        return backleft;
    }

    public DcMotor getRightMotorDown() {
        return backright;
    }

    @Deprecated
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public HardwareMap gethardwareMapy() {
        return hardwareMapy;
    }

    public void MoveForward(double speed) {

        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);

    }

    public void MoveOnSensor(int distance) {
        while (getDistance(DistanceUnit.CM) > getDistance(DistanceUnit.CM) - distance) {
            MoveForward(1);
        }
        MoveForward(0);
    }

    public enum Axis {X, Y, Z}

    public interface ObjectDetected {

        void onLeft();

        void onCenter();

        void onRight();

    }
}








