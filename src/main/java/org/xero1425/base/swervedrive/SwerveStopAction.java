package org.xero1425.base.swervedrive;

public class SwerveStopAction extends SwerveDriveAction {

    public SwerveStopAction(SwerveDriveSubsystem subsys)  {
        super(subsys) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().stop() ;
        setDone() ;
    }

    @Override
    public void run() {
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveStopAction" ;
        return ret ;
    }
}
