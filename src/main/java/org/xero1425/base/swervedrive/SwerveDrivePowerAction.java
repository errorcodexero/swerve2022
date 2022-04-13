package org.xero1425.base.swervedrive;

import org.xero1425.base.misc.XeroTimer;

public class SwerveDrivePowerAction extends SwerveDriveAction {
    private double angle_ ;
    private double power_ ;
    private XeroTimer timer_ ;

    public SwerveDrivePowerAction(SwerveDriveSubsystem subsys, double angle, double power, double duration) throws Exception {
        super(subsys) ;

        angle_ = angle ;
        power_ = power ;
        timer_ = new XeroTimer(subsys.getRobot(), "swerve-angle-power", duration) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().setAngleTarget(angle_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.FL, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.FR, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.BL, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.BR, power_) ;   
        
        timer_.start();
    }

    @Override
    public void run() {
        if (timer_.isExpired()) {
            try {
                getSubsystem().stop() ;
            }
            catch(Exception ex) {
            }

            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "SwerveDrivePowerAction " + power_ ;
    }
}

