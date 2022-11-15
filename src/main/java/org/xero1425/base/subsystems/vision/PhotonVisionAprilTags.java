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
    private double targetPose_ ;
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
                latency_ = nt_.getEntry("targetArea").getNumber(0.0).doubleValue() ;
                latency_ = nt_.getEntry("targetPitch").getNumber(0.0).doubleValue() ;
                latency_ = nt_.getEntry("targetPose").getNumber(0.0).doubleValue() ;
                latency_ = nt_.getEntry("targetYaw").getNumber(0.0).doubleValue() ;
                latency_ = nt_.getEntry("targetPixelsX").getNumber(0.0).doubleValue() ;
                latency_ = nt_.getEntry("targetPixelsY").getNumber(0.0).doubleValue() ;
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

    public double targetPose() {
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
}
