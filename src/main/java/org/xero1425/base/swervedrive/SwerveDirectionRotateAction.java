package org.xero1425.base.swervedrive;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDirectionRotateAction extends SwerveDriveAction {
    private Translation2d dir_ ;
    private double rot_ ;
    private double [] angles_ ;
    private double [] speeds_ ;
    private boolean dirty_ ;
    private double last_robot_angle_ ;
    private boolean scale_ ;
    private double duration_ ;
    private double start_ ;
    private double threshold_ ;

    private final double NearZero = 0.005 ;

    public SwerveDirectionRotateAction(SwerveDriveSubsystem subsys, double x, double y, double rot) {
        this(subsys, x, y, rot, Double.MAX_VALUE) ;
    }

    public SwerveDirectionRotateAction(SwerveDriveSubsystem subsys, double x, double y, double rot, double duration) {
        super(subsys) ;

        threshold_ = 0.5 ;
        dir_ = new Translation2d(x, y) ;
        rot_ = rot ;
        dirty_ = false ;
        duration_ = duration ;

        angles_ = new double[subsys.getModuleCount()] ;
        speeds_ = new double[subsys.getModuleCount()] ;

        //
        // These should both be true, but can be set to false to remove this processing from the
        // per robot loop calculations when trying to track down bugs.
        //
        // The scale_ flag controls scalling of the final set of velocities, if any module would exceed the maximum
        // velocityh a single module can reach.
        //
        scale_ = true ;

        for(int i = 0 ; i < subsys.getModuleCount() ; i++) {
            angles_[i] = 0.0 ;
            speeds_[i] = 0.0 ;
        }

    }

    public void updateTargets(double dirx, double diry, double rot) {
        if (dirx != dir_.getX() || diry != dir_.getY() || rot != rot_)
        {
            dir_ = new Translation2d(dirx, diry) ;

            rot_ = rot ;
            dirty_ = true ;

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("DirectionRotateAction update") ;
            logger.add("dirx", dirx) ;
            logger.add("diry", diry) ;
            logger.add("rot", rot) ;
            logger.endMessage();
        }
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        dirty_ = true ;
        start_ = getSubsystem().getRobot().getTime() ;
        last_robot_angle_ = getSubsystem().getAngle() ;
    }

    @Override
    public void run() {

        if (getSubsystem().getRobot().getTime() > start_ + duration_)
        {
            try {
                getSubsystem().stop() ;
            }
            catch(Exception ex) {
            }
            setDone() ;
        }

        double rangle = getSubsystem().getAngle() ;
        if (dirty_ || Math.abs(last_robot_angle_ - rangle) > threshold_) {
            calcModuleTrajectories() ;
            getSubsystem().setTargets(angles_, speeds_);
            last_robot_angle_ = rangle ;
            dirty_ = false ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
        try {
            getSubsystem().stop() ;
        }
        catch(Exception ex) {

        }
    }

    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveDriveDirectionRotateAction:" ;
        ret += " dx " + Double.toString(dir_.getX()) ;
        ret += " dy " + Double.toString(dir_.getY()) ;
        ret += " rot " + Double.toString(rot_) ;
        return ret ;
    }


    private void calcModuleTrajectories() {       
        MessageLogger logger  = getSubsystem().getRobot().getMessageLogger() ;

        if (Math.abs(dir_.getX()) < NearZero && Math.abs(dir_.getY()) < NearZero && Math.abs(rot_) < NearZero)
        {
            for(int i = 0 ; i < getSubsystem().getModuleCount() ; i++) {
                angles_[i] = getSubsystem().getModuleAngle(i) ;
                speeds_[i] = 0.0 ;
            }
        }
        else
        {
            double angle = getSubsystem().getAngle() ;
            Translation2d dirrot = rotateVector(dir_, -angle) ;
            double maxspeed = 0.0 ;
            for(int i = 0 ; i < getSubsystem().getModuleCount() ; i++) {
                Translation2d rotvec = createRotVector(i, rot_);
                Translation2d resvec = addVectors(dirrot, rotvec) ;
                double rads = Math.atan2(resvec.getY(), resvec.getX()) ;
                angles_[i] = Math.toDegrees(rads) ;
                speeds_[i] = resvec.getNorm() ;

                if (speeds_[i] > maxspeed)
                    maxspeed = speeds_[i] ;
            }

            if (scale_ && maxspeed > getSubsystem().getMaxSpeed()) {
                double scale = maxspeed / getSubsystem().getMaxSpeed() ;
                logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
                logger.add("Scale Velocity:") ;
                logger.add("scale", scale) ;
                logger.endMessage();
                for(int i = 0 ; i < speeds_.length ; i++) {
                    speeds_[i] /= scale ;
                }
            }
        }
    }

}
