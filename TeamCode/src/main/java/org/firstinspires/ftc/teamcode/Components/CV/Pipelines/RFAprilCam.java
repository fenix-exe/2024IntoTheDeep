package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import static java.lang.Math.sqrt;

import android.os.Build;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * Warren
 */
@Config
public class RFAprilCam {
    public static double X_OFFSET = 0, Y_OFFSET = 0, UPSAMPLE_THRESHOLD=50;
    public static float DOWNSAMPLE = 2, UPSAMPLE = 3;
    private AprilTagProcessor aprilTag;
    private RFVisionPortal visionPortal;
    boolean upsample=false;
    private Vector2d[] values = {new Vector2d(0, 0), new Vector2d(0, 0),new Vector2d(0, 0),new Vector2d(0, 0),
            new Vector2d(0, 0),new Vector2d(0, 0),new Vector2d(0, 0)
            ,new Vector2d(0, 0),new Vector2d(3*23.5-1,1.5*23.5),new Vector2d(3*23.5-1,1.5*23.5-4.5)
            ,new Vector2d(3*23.5-1,-1.5*23.5),new Vector2d( 3*23.5-1,-1.5*23.5+4.5)};
    private ArrayList<Pose2d> camPose = new ArrayList<>();
    private double[][] directions = {{-1, 1}, {-1, 1},{-1, 1},{-1, 1},{-1, 1},{-1, 1},{-1, 1},{-1, 1},{-1, -1},{-1, -1},{-1, -1}};

    public RFAprilCam() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();
        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE);
        aprilTag.setDecimation(DOWNSAMPLE);

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new RFVisionPortal.Builder()
                .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .setStreamFormat(RFVisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640, 480))
                .build();
        visionPortal.stopLiveView();
    }
    public boolean update() {
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        //if close start upsampling
        camPose.clear();
        for (AprilTagDetection detection : detections) {
            upsample=false;
             AprilTagPoseFtc poseFtc= detection.ftcPose;
             double p_x = poseFtc.y, p_y = poseFtc.x;
             int p_ind = detection.id;
             camPose.add(new Pose2d(values[p_ind].plus(new Vector2d(p_x * directions[p_ind][0]+X_OFFSET, p_y * directions[p_ind][1]+Y_OFFSET)),
                     directions[p_ind][0]*poseFtc.yaw));
             double dist = sqrt(poseFtc.x*poseFtc.x + poseFtc.y*poseFtc.y);
             if(dist<UPSAMPLE_THRESHOLD){
                 upsample=true;
             }
             logger.log("/RobotLogs/GeneralRobot", "aprilPos = "+camPose.get(camPose.size()-1)+", dist:"+dist);
        }
        if(upsample){
            aprilTag.setDecimation(UPSAMPLE);
        }else{
            aprilTag.setDecimation(DOWNSAMPLE);
        }
        return camPose.size() > 0 && upsample;
    }
    public Pose2d getCamPose(){
        if(camPose.size()>0) {
            return camPose.get(0);
        }
        else{
            return new Pose2d(0,0,0);
        }
    }
}
