package org.xero1425.base.swervedrive;

import org.xero1425.base.misc.XeroTimer;

public class SwerveDrivePowerAction extends SwerveDriveAction {
    private double angle_ ;
    private double power_ ;
    private XeroTimer timer_ ;

    private Double[] plot_data_ ;

    private int plotid_ ;
    static final private String [] columns_ = { "time", "fl (m/s)", "fr (m/s)", "bl (m/s)", "br (m/s)" } ;

    public SwerveDrivePowerAction(SwerveDriveSubsystem subsys, double angle, double power, double duration) throws Exception {
        super(subsys) ;

        angle_ = angle ;
        power_ = power ;
        timer_ = new XeroTimer(subsys.getRobot(), "swerve-angle-power", duration) ;

        plotid_ = subsys.initPlot("SwerveDrivePowerAction") ;
        plot_data_ = new Double[columns_.length] ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().startPlot(plotid_, columns_);

        getSubsystem().setAngleTarget(angle_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.FL, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.FR, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.BL, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.BR, power_) ;   
        
        timer_.start();
    }

    @Override
    public void run() {

        plot_data_[0] = timer_.elapsed() ;
        plot_data_[1] = getSubsystem().getModule(SwerveDriveSubsystem.FL).getSpeed() ;
        plot_data_[2] = getSubsystem().getModule(SwerveDriveSubsystem.FR).getSpeed() ;
        plot_data_[3] = getSubsystem().getModule(SwerveDriveSubsystem.BL).getSpeed() ;
        plot_data_[4] = getSubsystem().getModule(SwerveDriveSubsystem.BR).getSpeed() ;

        System.out.println("Plot " + plot_data_[0] + " " + plot_data_[2] + " " + plot_data_[2] + " " + plot_data_[3] + " " + plot_data_[4] + " ") ;
        getSubsystem().addPlotData(plotid_, plot_data_) ;

        if (timer_.isExpired()) {
            try {
                getSubsystem().stop() ;
            }
            catch(Exception ex) {
            }
            getSubsystem().endPlot(plotid_);
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

