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

    private TargetTrackerSubsystem tracker_ ;
    private DriveBaseSubsystem db_ ;
    private TurretSubsystem turret_ ;
    private GPMSubsystem sub_ ;

    private ShooterParams last_params_ ;
    private ShooterParams default_params_ ;

    private SetShooterAction shoot_action_ ;
    private ConveyorShootAction conveyor_action_ ;
    private boolean shooting_ ;

    private double db_max_velocity_ ;
    private double hood_threshold_ ;
    private double wheel_threshold_ ;

    private PieceWiseLinear pwl_hood_ ;
    private PieceWiseLinear pwl_velocity_ ;

    public GPMFireAction(GPMSubsystem gpm, TargetTrackerSubsystem tracker, DriveBaseSubsystem db, TurretSubsystem turret) throws Exception {
        super(gpm.getRobot().getMessageLogger()) ;

        sub_ = gpm ;
        tracker_ = tracker ;
        db_ = db ;
        turret_ = turret ;
        last_params_ = null ;

        db_max_velocity_ = gpm.getSettingsValue("fire-action:max-db-velocity").getDouble() ;
        hood_threshold_ = gpm.getSettingsValue("fire-action:hood-threshold").getDouble() ;
        wheel_threshold_ = gpm.getSettingsValue("fire-action:wheel-threshold").getDouble() ;

        double vel = gpm.getSettingsValue("fire-action:default-wheel-velocity").getDouble() ;
        double pos = gpm.getSettingsValue("fire-action:default-hood-position").getDouble() ;
        default_params_ = new ShooterParams(vel, pos, false) ;

        shoot_action_ = new SetShooterAction(gpm.getShooter(), 0.0, 0.0) ;
        conveyor_action_ = new ConveyorShootAction(gpm.getConveyor()) ;

        pwl_hood_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:gpm:fire-action:hood-pwl") ;
        pwl_velocity_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:gpm:fire-action:velocity-pwl") ;

        shooting_ = false ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        shooting_ = false ;
        sub_.getShooter().setAction(shoot_action_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        ConveyorSubsystem conveyor = sub_.getConveyor() ;

        if (shooting_) {
            //
            // We are already shooting.  We just wait for the conveyor to tell us we
            // are done.
            //
            if (conveyor.isIdle()) {
                shoot_action_.cancel() ;
                setDone() ;
            }
        }
        else {
            //
            // Set the shooter hood position and wheel velocity based on the target, or if a target
            // is not seen, based on the latest or default values.
            //
            ShooterParams sp = setShooterParams();

            //
            // See if everyone is ready to shoot, 
            //
            boolean shooter_ready = isShooterReady(sp) ;
            boolean db_ready = isDriveBaseReady() ;
            boolean turret_ready = turret_.isReadyToFire() ;

            sub_.putDashboard("sh-ready", DisplayType.Always, shooter_ready);
            sub_.putDashboard("db-ready", DisplayType.Always, db_ready);
            sub_.putDashboard("tu-ready", DisplayType.Always, turret_ready);

            if (shooter_ready && db_ready && turret_ready) {
                shoot_action_.startPlot();
                conveyor.setAction(conveyor_action_, true) ;
                shooting_ = true ;
            }
        }
    }

    private double getPercent(double target, double actual) {
        return Math.abs(target - actual) / target * 100.0 ;
    }

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
        }
        else {
            ShooterSubsystem shooter = sub_.getShooter() ;
            double hoodpcnt = getPercent(p.HoodPosition, shooter.getHoodSubsystem().getPosition());
            double wheelpcnt = getPercent(p.WheelVelocity, shooter.getWheelSubsystem().getVelocity()) ;

            sub_.putDashboard("hood-pcnt", DisplayType.Always, hoodpcnt);
            sub_.putDashboard("wheel-pcnt", DisplayType.Always, wheelpcnt);
            ret = hoodpcnt < hood_threshold_ && wheelpcnt < wheel_threshold_ ;        
            
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("GPMFireAction: isShooterReady: ") ;
            logger.add("hood", hoodpcnt) ;
            logger.add("wheel", wheelpcnt) ;
            logger.add("ready", ret) ;
            logger.endMessage();


        }

        return ret ;
    }

    private boolean isDriveBaseReady() {
        return db_.getVelocity() < db_max_velocity_ ;
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

        return new ShooterParams(wheel, hood, true) ;
    }

    private ShooterParams getDefaultShooterParams() {
        return default_params_ ;
    }
}
