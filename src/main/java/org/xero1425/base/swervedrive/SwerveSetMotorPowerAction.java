package org.xero1425.base.swervedrive;

public class SwerveSetMotorPowerAction extends SwerveDriveAction {
    private int which_ ;
    private double steer_power_ ;
    private double drive_power_ ;
    private double start_ ;
    private double duration_ ;
    private boolean timed_ ;

    public SwerveSetMotorPowerAction(SwerveDriveSubsystem subsys, int which, double steer, double drive) {
        super(subsys) ;

        which_ = which ;
        steer_power_ = steer ;
        drive_power_ = drive ;
        start_ = 0.0 ;
        duration_ = 0.0 ;
        timed_ = false ;
    }

    public SwerveSetMotorPowerAction(SwerveDriveSubsystem subsys, int which, double steer, double drive, double dur) {
        super(subsys) ;

        which_ = which ;
        steer_power_ = steer ;
        drive_power_ = drive ;
        start_ = 0.0 ;
        duration_ = dur ;
        timed_ = true ;
    }
    
    @Override
    public void start() throws Exception {
        super.start() ;

        try {
            getSubsystem().setPower(which_, steer_power_, drive_power_) ;
            if (timed_)
                start_ = getSubsystem().getRobot().getTime() ;
            else
                setDone() ;
        }
        catch(Exception ex) {
        }
    }

    @Override
    public void run() {
        if (timed_) {
            if (getSubsystem().getRobot().getTime() - start_ > duration_)
            {
                try 
                {
                    getSubsystem().setPower(which_, 0.0, 0.0) ;
                }
                catch(Exception ex)
                {
                }
                setDone() ;
            }
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        try {
            getSubsystem().setPower(which_, 0.0, 0.0) ;
        }
        catch(Exception ex)
        {
        }        
    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveDrivePowerAction" ;
        ret += " module='" + getSubsystem().getPairName(which_, true) + "'" ;
        ret += " steer=" + Double.toString(steer_power_) ;
        ret += " drive=" + Double.toString(drive_power_) ;
        if (timed_)
            ret += " duration=" + Double.toString(duration_) ;

        return ret ;
    }
}
