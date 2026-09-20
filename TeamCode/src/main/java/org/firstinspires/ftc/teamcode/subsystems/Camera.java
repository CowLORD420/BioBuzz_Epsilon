package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Camera {
    private final Limelight3A camGirl;
    private LLResult result;
    private List<LLResultTypes.FiducialResult> fiducials;

    public Camera(Limelight3A limelight3A){
        camGirl = limelight3A;
        camGirl.pipelineSwitch(0);
        camGirl.stop();
    }

    public void init(){
        camGirl.start();
    }

    public void update(){
        result = camGirl.getLatestResult();
        fiducials = result.getFiducialResults();
    }

    public void stop(){
        camGirl.stop();
    }

    public void updateYaw(double yawInDegrees){
        camGirl.updateRobotOrientation(yawInDegrees);
    }

    //inverted because Tx is -to the right in the camera frame
    public boolean hasTarget() {
        return fiducials != null && !fiducials.isEmpty() && result != null && result.isValid();
    }

    public double getError() {
        if (hasTarget()) {
            return -result.getTx();
        }
        return 0; // caller must check hasTarget() before trusting this
    }

    //call at init throwException if no april tag
    public Pose2D getBotpose(){
        if (fiducials != null) {
            if (result.isValid()) {
                return pose3Dto2D(
                        result.getBotpose_MT2()
                );
            }
        }
        throw new RuntimeException("No scan");
    }

    public int getTagID(){
        return fiducials.get(0).getFiducialId();
    }

    public void switchPipeline(int pipeline){
        camGirl.pipelineSwitch(pipeline);
    }

    public Pose2D pose3Dto2D(Pose3D pose3D){
        return new Pose2D(pose3D.getPosition().unit, pose3D.getPosition().x, pose3D.getPosition().y, AngleUnit.RADIANS, pose3D.getOrientation().getYaw(AngleUnit.RADIANS));
    }

}
