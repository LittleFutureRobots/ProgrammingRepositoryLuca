package org.firstinspires.ftc.teamcode.ProgrammingFTC2019LFR;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class AutonomousMakers {

    private final String VUFORIA_KEY = "AWIKGT7/////AAABmS3bsjJazEclpQEbA+BwmGRv5e1zEDZUgBVwvH+PniaFdrTOn96jCmryjubsOJdYXcsAkilFSWI4OSu4CNdB/AG6Q4JKSdwrvYwQIQ6H03QXP2$";
    public HardwareMap hwmap;
    Telemetry telemetry;
    private TFObjectDetector[] tfods = new TFObjectDetector[3];
    private HardwareMap hardwareMapy;
    private DcMotor leftMotorUp, rightMotorUp, leftMotorDown, rightMotorDown, motorarm0;
    private Servo servo3;
    private BNO055IMU imu;
    private VuforiaLocalizer[] cams = new VuforiaLocalizer[3];
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackables targetsRoverRuckus;
    private OpenGLMatrix lastLocation;
    private boolean targetVisible;
    private HashMap<String, Thread> running = new HashMap<>();
    private boolean emergencyStop = false;

    public void initAll(HardwareMap hardwareMapy) {
        hwmap = hardwareMapy;
        initGyro(BNO055IMU.AngleUnit.DEGREES);
        initRobot();
    }

    public void initRobot() {

        leftMotorUp = hwmap.get(DcMotor.class, "frontleft");
        rightMotorUp = hwmap.get(DcMotor.class, "frontright");
        leftMotorDown = hwmap.get(DcMotor.class, "backleft");
        rightMotorDown = hwmap.get(DcMotor.class, "backright");
        servo3 = hwmap.get(Servo.class, "servo3");
        motorarm0 = hwmap.get(DcMotor.class, "motorarm0");

    }

    public void disable() {
        for (int i = 0; i < tfods.length; i++) {
            if (tfods[i] != null) {
                tfods[i].shutdown();
            }
        }
        emergencyStop();
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
                    turnTo(0.3, 30);
                } else {
                    turnTo(0.3, -30);
                }
            }
        }
    }

    public void keepPosition(int cameraIndex) {
        TFObjectDetector tf = tfods[cameraIndex];
        tf.activate();

        List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
        while (true) {
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() != 0) {
                    int x = (int) updatedRecognitions.get(0).getLeft();
                    if (x > 400) {
                        getLeftMotorUp().setPower(0.3);
                        getRightMotorUp().setPower(0.3);
                        getLeftMotorDown().setPower(0.3);
                        getRightMotorDown().setPower(0.3);
                    } else {
                        getLeftMotorUp().setPower(-0.3);
                        getRightMotorUp().setPower(-0.3);
                        getLeftMotorDown().setPower(-0.3);
                        getRightMotorDown().setPower(-0.3);
                    }

                    getLeftMotorUp().setPower(0);
                    getRightMotorUp().setPower(0);
                    getLeftMotorDown().setPower(0);
                    getRightMotorDown().setPower(0);
                }
            }
            updatedRecognitions = tf.getUpdatedRecognitions();
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
    public void emergencyStop() {
        emergencyStop = true;
        for (String name : running.keySet()) {
            running.remove(name).stop();
        }
    }

    public void activateTrackable() {
        targetsRoverRuckus.activate();
    }

    public Location getLocation() {

        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
        }
        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return new Location(translation.get(0) / 25.4f, translation.get(1) / 25.4f, translation.get(2) / 25.4f, rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        }


        return new Location(-1);
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
        return leftMotorUp;
    }

    public DcMotor getRightMotorUp() {
        return rightMotorUp;
    }

    public DcMotor getLeftMotorDown() {
        return leftMotorDown;
    }

    public DcMotor getRightMotorDown() {
        return rightMotorDown;
    }

    @Deprecated
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public HardwareMap gethardwareMapy() {
        return hardwareMapy;
    }

    public void DriveForward(double speed) {
        leftMotorUp.setPower(speed);
        leftMotorDown.setPower(speed);
        rightMotorUp.setPower(speed);
        rightMotorDown.setPower(speed);
    }

    public void LanderArm(double speed) {
        motorarm0.setPower(speed);
    }

    public void TeamMarker(double position) {
        servo3.setPosition(position);
    }

    public enum Axis {X, Y, Z}

    public enum TeamSide {
        RIGHT(135), LEFT(225), UNKNOWN(0);

        private int angle;

        TeamSide(int angle) {
            this.angle = angle;
        }

        public int getAngle() {
            return angle;
        }
    }

    public interface ObjectDetected {

        void onLeft();

        void onCenter();

        void onRight();

    }

    public class Location {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float heading;
        int status = 1;

        public Location(int status) {
            this.status = status;
        }

        public Location(float x, float y, float z, float roll, float pitch, float heading) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.heading = heading;
        }

        public float getX() {
            return x;
        }

        public float getY() {
            return y;
        }

        public float getZ() {
            return z;
        }

        public float getRoll() {
            return roll;
        }

        public float getPitch() {
            return pitch;
        }

        public float getHeading() {
            return heading;
        }

        public void print() {
            if (status != -1) {
                telemetry.addData("Position(X, Y, Z)", "%.00f, %.00f, %.00f", x, y, z);
                telemetry.addData("Position(Roll, Pitch, Heading)", "%.00f, %.00f, %.00f", roll, pitch, heading);
            }
            telemetry.update();
        }
    }


}


