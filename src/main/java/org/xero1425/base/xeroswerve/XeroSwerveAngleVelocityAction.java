package org.xero1425.base.xeroswerve;

import org.xero1425.base.misc.XeroTimer;

public class XeroSwerveAngleVelocityAction extends XeroSwerveDriveAction {
    private double [] angles_ ;
    private double [] speeds_ ;
    private boolean hold_ ;

    private XeroTimer plot_timer_ ;
    private int plotid_ ;
    private Double [] plot_data_ ;
    private final static String columns_[] = { 
        "time", 
        "flspeed (m/s)", "flangle (degrees)", "fltarget (m/s)",
        "frspeed (m/s)", "frangle (degrees)", "frtarget (m/s)",
        "blspeed (m/s)", "blangle (degrees)", "bltarget (m/s)",
        "brspeed (m/s)", "brangle (degrees)", "brtarget (m/s)",
        "rot (deg)", "rotspeed (deg/s)"
    } ;

    public XeroSwerveAngleVelocityAction(XeroSwerveDriveSubsystem subsys, double [] angles, double [] speeds, boolean hold) throws Exception {
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

        plot_timer_ = new XeroTimer(subsys.getRobot(), "SwerveAngleVelocityAction-plot-timer", 4) ;
        plot_data_ = new Double[columns_.length] ;
        plotid_ = subsys.initPlot("SwerveAngleVelocityAction") ;
    }

    public XeroSwerveAngleVelocityAction(XeroSwerveDriveSubsystem subsys, double angle, double speeds) throws Exception {
        super(subsys) ;

        angles_ = new double[4] ;
        angles_[XeroSwerveDriveSubsystem.FL] = createRotAngle(XeroSwerveDriveSubsystem.FL) ;
        angles_[XeroSwerveDriveSubsystem.BL] = createRotAngle(XeroSwerveDriveSubsystem.BL) ;
        angles_[XeroSwerveDriveSubsystem.FR] = createRotAngle(XeroSwerveDriveSubsystem.FR) ;
        angles_[XeroSwerveDriveSubsystem.BR] = createRotAngle(XeroSwerveDriveSubsystem.BR) ;

        speeds_ = new double[4] ;
        for(int i = 0 ; i < 4 ; i++) 
            speeds_[i] = speeds ;

        plot_timer_ = new XeroTimer(subsys.getRobot(), "SwerveAngleVelocityAction-plot-timer", 4) ;
        plot_data_ = new Double[columns_.length] ;
        plotid_ = subsys.initPlot("SwerveAngleVelocityAction") ;
    }

    void updateTargets(double[] angles, double[] speeds) {
        angles_ = angles.clone() ;
        speeds_ = speeds.clone() ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        plot_timer_.start();
        getSubsystem().startPlot(plotid_, columns_);

        getSubsystem().setTargets(angles_, speeds_) ;
    }

    @Override
    public void run() {
        getSubsystem().setTargets(angles_, speeds_) ;

        if (plot_timer_.isExpired()) {
            getSubsystem().endPlot(plotid_);
        }
        else {
            int index = 0 ;
            plot_data_[index++] = plot_timer_.elapsed() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.FL).getSpeed() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.FL).getAngle() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.FL).getSpeedTarget() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.FR).getSpeed() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.FR).getAngle() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.FR).getSpeedTarget() ;            
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.BL).getSpeed() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.BL).getAngle() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.BL).getSpeedTarget() ;            
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.BR).getSpeed() ;
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.BR).getAngle() ;           
            plot_data_[index++] = getSubsystem().getModule(XeroSwerveDriveSubsystem.BR).getSpeedTarget() ;
            plot_data_[index++] = getSubsystem().getAngle();
            plot_data_[index++] = getSubsystem().getAngleSpeed() ;
            getSubsystem().addPlotData(plotid_, plot_data_);
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        getSubsystem().endPlot(plotid_);

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
