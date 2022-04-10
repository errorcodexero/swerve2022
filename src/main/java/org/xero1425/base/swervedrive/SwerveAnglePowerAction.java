package org.xero1425.base.swervedrive;

public class SwerveAnglePowerAction extends SwerveDriveAction {
    private double angle_ ;
    private double power_ ;
    private double start_ ;
    private double duration_ ;

    private int plot_id_ ;
    private static int plot_number_ = 0 ;
    private static final String [] plot_columns_ = { "time", "dist", "velocity", "acceleration"} ;

    public SwerveAnglePowerAction(SwerveDriveSubsystem subsys, double angle, double power, double duration) throws Exception {
        super(subsys) ;

        angle_ = angle ;
        power_ = power ;
        duration_ = duration ;
        plot_id_ = subsys.initPlot("swerve" + Integer.toString(plot_number_++)) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().setAngleTarget(angle_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.FL, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.FR, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.BL, power_) ;
        getSubsystem().setDriveMotorPower(SwerveDriveSubsystem.BR, power_) ;                        

        start_ = getSubsystem().getRobot().getTime() ;

        getSubsystem().startPlot(plot_id_, plot_columns_);
    }

    @Override
    public void run() {
        if (getSubsystem().getRobot().getTime() - start_ > duration_) {
            try {
                getSubsystem().stop() ;

            }
            catch(Exception ex) {

            }

            setDone() ;
            getSubsystem().endPlot(plot_id_) ;
        }
        else {
            Double[] data = new Double[plot_columns_.length] ;
            data[0] = getSubsystem().getRobot().getTime() - start_ ;
            data[1] = getSubsystem().getDistance() ;
            data[2] = getSubsystem().getVelocity() ;
            data[3] = getSubsystem().getAcceleration() ;
            getSubsystem().addPlotData(plot_id_, data);
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveAnglePowerAction:" ;
        ret += " " + Double.toString(angle_) ;
        ret += " " + Double.toString(power_) ;
        ret += " " + Double.toString(duration_) ;
        return ret ;
    }
}
