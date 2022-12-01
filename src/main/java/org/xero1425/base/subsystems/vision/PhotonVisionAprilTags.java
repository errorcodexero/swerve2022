package org.xero1425.base.subsystems.vision;

import org.xero1425.base.subsystems.Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonVisionAprilTags extends Subsystem {
    private boolean connected_ ;
    private boolean hasTarget_ ;
    private double latency_ ;
    private double targetArea_ ;
    private double targetPitch_ ;
    private double[] targetPose_ ;

    private double targetSkew_;
    private double targetYaw_ ;
    private double targetPixelsX_ ;
    private double targetPixelsY_ ;

    private String camname_ ;
    private NetworkTable nt_ ;

    public PhotonVisionAprilTags(Subsystem parent, String name, String camname) {
        super(parent, name) ;

        camname_ = camname ;
        hasTarget_ = false ;

        String tabname = "photonvision/" + camname_ ;

        nt_ = NetworkTableInstance.getDefault().getTable(tabname) ;
    }

    @Override
    public void computeMyState() {
        if (nt_.containsKey("hasTarget"))
        {
            connected_ = true ;
            hasTarget_ = nt_.getEntry("hasTarget").getBoolean(false) ;
            if (hasTarget_) {
                latency_ = nt_.getEntry("latencyMillis").getNumber(0.0).doubleValue() ;
                targetArea_ = nt_.getEntry("targetArea").getNumber(0.0).doubleValue() ;
                targetPitch_ = nt_.getEntry("targetPitch").getNumber(0.0).doubleValue() ;
                targetPose_ = nt_.getEntry("targetPose").getDoubleArray(new double[3]);
                targetYaw_ = nt_.getEntry("targetYaw").getNumber(0.0).doubleValue() ;
                targetSkew_ = nt_.getEntry("targetSkew").getNumber(0.0).doubleValue() ;
                targetPixelsX_ = nt_.getEntry("targetPixelsX").getNumber(0.0).doubleValue() ;
                targetPixelsY_ = nt_.getEntry("targetPixelsY").getNumber(0.0).doubleValue() ;
            }
        }
        else
        {
            connected_ = false ;
        }
    }

    public boolean isConnected() {
        return connected_ ;
    }

    public boolean hasTarget() {
        return hasTarget_ ;
    }

    public double latency() {
        return latency_ ;
    }

    public double targetArea() {
        return targetArea_ ;
    }

    public double targetPitch() {
        return targetPitch_ ;
    }

    public double[] targetPose() {
        return targetPose_ ;
    }

    public double targetYaw() {
        return targetYaw_ ;
    }

    public double targetPixelsX() {
        return targetPixelsX_ ;
    }

    public double targetPixelsY() {
        return targetPixelsY_ ;
    }
    public double targetSkew() {
        return targetSkew_ ;
    }

    public void setPipelineIndex(int index) {
        nt_.getEntry("pipelineIndex").setNumber(index);
    }
    public int getPipelineIndex() {
        return nt_.getEntry("pipelineIndex").getNumber(0).intValue();
    }

    public void setDriverMode(boolean driverMode) {
        nt_.getEntry("driverMode").setBoolean(driverMode);
    }
    public boolean getDriverMode() {
        return nt_.getEntry("driverMode").getBoolean(false);
    }

    public void setInputSaveImgCmd(boolean saveImg) {
        nt_.getEntry("inputSaveImgCmd").setBoolean(saveImg);
    }
    public boolean getInputSaveImgCmd() {
        return nt_.getEntry("inputSaveImgCmd").getBoolean(false);
    }


    public void setOutputSaveImgCmd(boolean saveImg) {
        nt_.getEntry("outputSaveImgCmd").setBoolean(saveImg);
    }
    public boolean getOutputSaveImgCmd() {
        return nt_.getEntry("outputSaveImgCmd").getBoolean(false);
    }
}
