package org.xero1425.base.swervedrive;

public class SwerveAngleVelocityAction extends SwerveDriveAction {
    private double [] angles_ ;
    private double [] speeds_ ;
    private boolean hold_ ;

    public SwerveAngleVelocityAction(SwerveDriveSubsystem subsys, double [] angles, double [] speeds, boolean hold) throws Exception {
        super(subsys) ;

        if (angles.length != 4) {
            throw new Exception("SwerveAngleVelocityAction action with angles.length not equal four.") ;
        }

        if (speeds.length != 4) {
            throw new Exception("SwerveAngleVelocityAction action with angles.length not equal four.") ;
        }

        angles_ = angles.clone() ;
        speeds_ = speeds.clone() ;
        hold_ = hold ;
    }

    void updateTargets(double[] angles, double[] speeds) {
        angles_ = angles.clone() ;
        speeds_ = speeds.clone() ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().setTargets(angles_, speeds_) ;
    }

    @Override
    public void run() {
        getSubsystem().setTargets(angles_, speeds_) ;
    }

    @Override
    public void cancel() {
        super.cancel() ;

        if (hold_ == false)
        {
            try {
                getSubsystem().stop() ;
            }
            catch(Exception ex) {                
            }
        }

    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveAngleVelocityAction: " ;
        for(int which = 0 ; which < getSubsystem().getModuleCount()  ; which++) 
        {
            if (which != 0)
                ret += " " ;
            ret += getSubsystem().getPairName(which, true) ;
            ret += " " + Double.toString(angles_[which]) ;
            ret += " " + Double.toString(speeds_[which]) ;
        }
        return ret ;
    }
}
