package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.List;
// Todo make aprilTags point to middle of the balansoar type shii
// Red is pipeline 0 blue is pipeline 1
public class Camera {
    private final Limelight3A camGirl;
    private LLResult result;
    private List<LLResultTypes.FiducialResult> fiducials;

    public Camera(Limelight3A limelight3A, Robot.Color color){
        camGirl = limelight3A;
        camGirl.stop();
        switch (color){
            case RED:
                camGirl.pipelineSwitch(0);
                break;
            case BLUE:
                camGirl.pipelineSwitch(1);
                break;
        }
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

    public void updateYawInDegrees(double yawInDegrees){
        camGirl.updateRobotOrientation(yawInDegrees);
    }

    //inverted because Tx is -to the right in the camera frame
    public boolean hasTarget() {
        return fiducials != null && !fiducials.isEmpty() && result != null && result.isValid();
    }

    public double getErrorInDegrees() {
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
