package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem.DisplayType;
import org.xero1425.base.utils.PieceWiseLinear;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.conveyor.ConveyorShootAction;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.shooter.SetShooterAction;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class GPMFireAction extends Action {

    private class ShooterParams {
        public final double WheelVelocity ;
        public final double HoodPosition ;
        public final boolean IsValid ;

        public ShooterParams(double wheel, double hood, boolean valid) {
            WheelVelocity = wheel ;
            HoodPosition = hood ;
            IsValid = valid ;
        }
    }

    private static int count_ = 1 ;

    private TargetTrackerSubsystem tracker_ ;
    private DriveBaseSubsystem db_ ;
    private TurretSubsystem turret_ ;
    private GPMSubsystem sub_ ;

    private ShooterParams last_params_ ;
    private ShooterParams default_params_ ;

    private SetShooterAction shoot_action_ ;
    private ConveyorShootAction conveyor_action_ ;
    private boolean shooting_ ;

    private double db_max_linear_velocity_ ;
    private double db_max_rotational_velocity_ ;
    private double hood_threshold_ ;
    private double wheel_threshold_ ;
    private double turret_factor_ ;

    private PieceWiseLinear pwl_hood_ ;
    private PieceWiseLinear pwl_velocity_ ;

    private boolean hood_ready_ ;
    private boolean wheels_ready_ ;
    private boolean target_ready_ ;
    private boolean turret_ready_ ;

    private double start_ ;
    private int plot_id_ ;
    private Double[] data_ ;

    private int howmany_ ;
    private int howmany_threshold_ ;

    private static String[] plot_columns_ = { "time", "distance (in)", "sact (rpm)", "hact (enc)", "starget (rpm)", "htarget (enc)", "fire" } ;

    public GPMFireAction(GPMSubsystem gpm, TargetTrackerSubsystem tracker, DriveBaseSubsystem db, TurretSubsystem turret) throws Exception {
        super(gpm.getRobot().getMessageLogger()) ;

        sub_ = gpm ;
        tracker_ = tracker ;
        db_ = db ;
        turret_ = turret ;
        last_params_ = null ;

        db_max_linear_velocity_ = gpm.getSettingsValue("fire-action:max-db-linear-velocity").getDouble() ;
        db_max_rotational_velocity_ = gpm.getSettingsValue("fire-action:max-db-rotational-velocity").getDouble() ;
        hood_threshold_ = gpm.getSettingsValue("fire-action:hood-threshold").getDouble() ;
        wheel_threshold_ = gpm.getSettingsValue("fire-action:wheel-threshold").getDouble() ;
        turret_factor_ = gpm.getSettingsValue("fire-action:turret-factor").getDouble() ;

        double vel = gpm.getSettingsValue("fire-action:default-wheel-velocity").getDouble() ;
        double pos = gpm.getSettingsValue("fire-action:default-hood-position").getDouble() ;
        default_params_ = new ShooterParams(vel, pos, false) ;

        howmany_threshold_ = gpm.getSettingsValue("fire-action:how-many-threshold").getInteger() ;

        shoot_action_ = new SetShooterAction(gpm.getShooter(), 0.0, 0.0) ;
        conveyor_action_ = new ConveyorShootAction(gpm.getConveyor()) ;

        pwl_hood_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:gpm:fire-action:hood-pwl") ;
        pwl_velocity_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:gpm:fire-action:velocity-pwl") ;

        shooting_ = false ;

        data_ = new Double[plot_columns_.length] ;


        howmany_ = 0 ;
    }

    public boolean shooting() {
        return shooting_ ;
    }

    public boolean turretReady() {
        return turret_ready_ ;
    }

    public boolean targetReady() {
        return target_ready_ ;
    }

    public boolean shooterWheelsReady() {
        return wheels_ready_ ;
    }

    public boolean shooterHoodReady() {
        return hood_ready_ ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        shooting_ = false ;
        sub_.getShooter().setAction(shoot_action_, true) ;

        start_ = sub_.getRobot().getTime() ;
        plot_id_ = sub_.initPlot("shoot-" + count_) ;
        count_++ ;
        sub_.startPlot(plot_id_, plot_columns_);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        ConveyorSubsystem conveyor = sub_.getConveyor() ;
        ShooterParams sp = setShooterParams();

        int i = 0 ;
        data_[i++] = sub_.getRobot().getTime() - start_ ;
        data_[i++] = tracker_.getDistance() ;
        data_[i++] = sub_.getShooter().getWheelSubsystem().getVelocity() ;
        data_[i++] = sub_.getShooter().getHoodSubsystem().getPosition() ;
        data_[i++] = sp.WheelVelocity ;
        data_[i++] = sp.HoodPosition ;

        if (shooting_) {
            //
            // We are already shooting.  We just wait for the conveyor to tell us we
            // are done.
            //
            if (conveyor.isIdle()) {
                MessageLogger logger = sub_.getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
                logger.add("GPMFireAction: Shooting dones") ;
                logger.endMessage();
                
                shoot_action_.cancel() ;
                sub_.endPlot(plot_id_);
                setDone() ;
            }

            data_[i++] = 1.0 ;
        }
        else {
            //
            // Set the shooter hood position and wheel velocity based on the target, or if a target
            // is not seen, based on the latest or default values.
            //

            //
            // See if everyone is ready to shoot, 
            //
            boolean shooter_ready = isShooterReady(sp) ;
            boolean db_ready = isDriveBaseReady() ;
            turret_ready_ = turret_.isReadyToFire() ;

            sub_.putDashboard("sh-ready", DisplayType.Always, shooter_ready);
            sub_.putDashboard("db-ready", DisplayType.Always, db_ready);
            sub_.putDashboard("tu-ready", DisplayType.Always, turret_ready_);

            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("GPMFireAction: ") ;
            logger.add("wheels " + wheels_ready_) ;
            logger.add(", hood " + hood_ready_) ;
            logger.add(", db " + db_ready) ;
            logger.add(", turret " + turret_ready_) ;
            logger.endMessage();

            if (shooter_ready && db_ready && turret_ready_) {
                howmany_++ ;

                if (howmany_ > howmany_threshold_) {
                    logger.startMessage(MessageType.Debug) ;
                    logger.add("GPMFireAction: Shooting start") ;
                    logger.add(", distance", tracker_.getDistance());
                    logger.endMessage();
    
                    shoot_action_.startPlot();
                    conveyor.setAction(conveyor_action_, true) ;
                    shooting_ = true ;
                }
            }
            else {
                howmany_ = 0 ;
            }

            data_[i++] = 0.0 ;

            if (data_[0] > 10.0) {
                sub_.endPlot(plot_id_);
            }
        }

        sub_.addPlotData(plot_id_, data_);
    }

    // private double getPercent(double target, double actual) {
    //     return Math.abs(target - actual) / target * 100.0 ;
    // }

    private boolean isShooterReady(ShooterParams p) {
        boolean ret = false ;

        if (!p.IsValid) {
            //
            // If the shooting parameters are not valid, that is they are not based on the
            // limelight seeing the target, then the shooter will never be ready.
            //
            sub_.putDashboard("hood-pcnt", DisplayType.Always, Double.NaN);
            sub_.putDashboard("wheel-pcnt", DisplayType.Always, Double.NaN);
            ret = false ;

            hood_ready_ = false ;
            wheels_ready_ = false ;
        }
        else {
            ShooterSubsystem shooter = sub_.getShooter() ;
            double hooderr = Math.abs(p.HoodPosition - shooter.getHoodSubsystem().getPosition()) ;
            double wheelerr = Math.abs(p.WheelVelocity - shooter.getWheelSubsystem().getVelocity()) ;

            sub_.putDashboard("hood-pcnt", DisplayType.Always, hooderr);
            sub_.putDashboard("wheel-pcnt", DisplayType.Always, wheelerr);

            hood_ready_ = hooderr < hood_threshold_ ;
            wheels_ready_ = wheelerr < wheel_threshold_ ;

            ret = hood_ready_ && wheels_ready_ ;
            
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("GPMFireAction: isShooterReady: ") ;
            logger.add("hood-pcnt", hooderr) ;
            logger.add("hood-pos", shooter.getHoodSubsystem().getPosition()) ;
            logger.add("hood-target", p.HoodPosition) ;
            logger.add("wheel-pcnt", wheelerr) ;
            logger.add("wheel-vel", shooter.getWheelSubsystem().getVelocity()) ;
            logger.add("wheel-target", p.WheelVelocity) ;
            logger.add("ready", ret) ;
            logger.endMessage();
        }

        return ret ;
    }

    private boolean isDriveBaseReady() {
        return db_.getVelocity() < db_max_linear_velocity_  && db_.getRotationalVelocity() < db_max_rotational_velocity_ ;
    }

    private ShooterParams setShooterParams() throws BadMotorRequestException, MotorRequestFailedException {
        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        ShooterParams sp ;

        if (tracker_.hasTarget()) {
            sp = computeShooterParams(tracker_.getDistance()) ;
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("GPMFireAction: has target") ;
            logger.add("hood", sp.HoodPosition) ;
            logger.add("wheel", sp.WheelVelocity) ;
            logger.endMessage();
            last_params_ = new ShooterParams(sp.WheelVelocity, sp.HoodPosition, false) ;
        }
        else if (last_params_ != null) {
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("GPMFireAction: no target, has previous params") ;
            logger.endMessage();
            sp = last_params_ ;
        }
        else {
            sp = getDefaultShooterParams() ;
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("GPMFireAction: no target, default params") ;
            logger.add("hood", sp.HoodPosition) ;
            logger.add("wheel", sp.WheelVelocity) ;
            logger.endMessage();
        }

        if (sp.IsValid)
            target_ready_ = true ;
        else
            target_ready_ = false ;

        shoot_action_.update(sp.WheelVelocity, sp.HoodPosition) ;
        return sp ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMFireAction" ;
    }

    private ShooterParams computeShooterParams(double dist) {
        double hood = pwl_hood_.getValue(dist) ;
        double wheel = pwl_velocity_.getValue(dist) ;

        double mult = Math.abs(turret_.getPosition()) / 90.0 * turret_factor_ ;
        wheel *= (1.0 + mult) ;

        // hood += 175 ;
        // wheel *= 1.04 ;

        if (hood > 900) {
            hood = 900 ;
        }

        return new ShooterParams(wheel, hood, true) ;
    }

    private ShooterParams getDefaultShooterParams() {
        return default_params_ ;
    }
}
