package frc.robot.subsystems.shooter;


import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderTrackPositionAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderTrackVelocityAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class SetShooterAction extends Action {
    private ShooterSubsystem sub_;
    private MotorEncoderTrackVelocityAction wheel_action_;
    private MotorEncoderTrackPositionAction hood_action_;

    private static int plot_number_ = 0 ;

    private int plot_id_ ;
    private double plot_start_ ;
    private Double [] plot_data_ ;

    private double wheel_ ;
    private double hood_ ;

    // The columns to plot
    private static String [] columns_ = { "time", "ltarget(rpm)", "lactual(rpm)", "htarget(degrees)", "hactual(degrees)" } ;

    public SetShooterAction(ShooterSubsystem sub, double wheels, double hood) throws Exception
    {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        wheel_action_ = new MotorEncoderTrackVelocityAction(sub.getWheelSubsystem(), "wheels", wheels);
        hood_action_ = new MotorEncoderTrackPositionAction(sub.getHoodSubsystem(), "hoodpos", hood);

        plot_id_ = -1 ;
        plot_data_ = new Double[7] ;

        wheel_ = Double.NaN ;
        hood_ = Double.NaN ;
    }

    public void update(double wheel, double hood) throws BadMotorRequestException, MotorRequestFailedException {

        String change ="" ;

        if (wheel_ == Double.NaN || wheel_ != wheel) {
            wheel_action_.setTarget(wheel);
            change += "wheel: " + Double.toString(wheel_) + " --> " + Double.toString(wheel) ;
            wheel_ = wheel ;
        }

        if (hood_ == Double.NaN || hood != hood_) {
            hood_action_.setTarget(hood);
            if (change.length() > 0) {
                change += "  " ;
            }
            change += "hood: " + Double.toString(hood_) + " --> " + Double.toString(hood) ;
            hood_ = hood ;
        }

        if (change.length() > 0) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("SetShooterAction: update: ").add(change).endMessage();
        }
    }

    public void startPlot() {
        plot_id_ = sub_.initPlot("SetShooterAction-" + plot_number_++) ;
        sub_.startPlot(plot_id_, columns_);
        plot_start_ = sub_.getRobot().getTime() ;
    }

    public void stopPlot() {
        sub_.endPlot(plot_id_) ;
        plot_id_ = -1 ;
    }

    @Override
    public void start() throws Exception{
        super.start();
        sub_.getWheelSubsystem().setAction(wheel_action_, true);
        sub_.getHoodSubsystem().setAction(hood_action_, true);
    }

    @Override
    public void run() throws Exception{
        super.run();

        plot_data_[0] = sub_.getRobot().getTime() - plot_start_ ;
        plot_data_[1] = wheel_action_.getTarget() ;
        plot_data_[2] = sub_.getWheelSubsystem().getVelocity() ;
        plot_data_[3] = hood_action_.getTarget() ;
        plot_data_[4] = sub_.getHoodSubsystem().getPosition() ;
        sub_.addPlotData(plot_id_, plot_data_) ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SetShooterAction vel " + wheel_action_.getTarget() + " " +  hood_action_.getTarget() ; 
    }

    @Override
    public void cancel() {
        super.cancel() ;
        sub_.getWheelSubsystem().cancelAction();
        sub_.getHoodSubsystem().cancelAction() ;
    }
}