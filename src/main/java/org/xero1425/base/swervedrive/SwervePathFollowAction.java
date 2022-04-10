package org.xero1425.base.swervedrive;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

public class SwervePathFollowAction extends SwerveDriveAction {
    private int index_;
    private String pathname_;
    private XeroPath path_;
    private double[] angles_;
    private double[] speeds_;
    private double start_dist_ ;

    private final int FL = 0 ;
    private final int FR = 1 ;
    private final int BL = 2 ;
    private final int BR = 3 ;

    public SwervePathFollowAction(SwerveDriveSubsystem drive, String path) {

        super(drive);

        pathname_ = path;
        path_ = null;

        angles_ = new double[4] ;
        speeds_ = new double[4] ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        start_dist_ = getSubsystem().getDistance() ;

        getSubsystem().startSwervePlot();

        index_ = 0;
        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        if (index_ < path_.getSize())
        {
            XeroPathSegment fl = path_.getSegment(FL, index_) ;
            XeroPathSegment fr = path_.getSegment(FR, index_) ;
            XeroPathSegment bl = path_.getSegment(BL, index_) ;
            XeroPathSegment br = path_.getSegment(BR, index_) ;

            angles_[SwerveDriveSubsystem.FL] = fl.getHeading() ;
            angles_[SwerveDriveSubsystem.FR] = fr.getHeading() ;
            angles_[SwerveDriveSubsystem.BL] = bl.getHeading() ;
            angles_[SwerveDriveSubsystem.BR] = br.getHeading() ;

            speeds_[SwerveDriveSubsystem.FL] = fl.getVelocity() ;
            speeds_[SwerveDriveSubsystem.FR] = fr.getVelocity() ;
            speeds_[SwerveDriveSubsystem.BL] = bl.getVelocity() ;
            speeds_[SwerveDriveSubsystem.BR] = br.getVelocity() ;

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("Assigned Data") ;
            logger.add("fl").add(fl.getHeading()) ;
            logger.add("fr").add(fr.getHeading()) ;
            logger.add("bl").add(bl.getHeading()) ;
            logger.add("br").add(br.getHeading()) ;
            logger.endMessage();

            getSubsystem().setTargets(angles_, speeds_);

            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("Assigned Data") ;
            logger.add("fl").add(getSubsystem().getModule(SwerveDriveSubsystem.FL).getAngle()) ;
            logger.add("fr").add(getSubsystem().getModule(SwerveDriveSubsystem.FR).getAngle()) ;
            logger.add("bl").add(getSubsystem().getModule(SwerveDriveSubsystem.BL).getAngle()) ;
            logger.add("br").add(getSubsystem().getModule(SwerveDriveSubsystem.BR).getAngle()) ;
            logger.endMessage();

            index_++ ;
        }

        if (index_ == path_.getSize())
        {
            getSubsystem().endSwervePlot();
            getSubsystem().stop();
            setDone() ;

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("Path Following") ;
            logger.add("distance", getSubsystem().getDistance() - start_dist_) ;
            logger.endMessage();
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
        setDone();
    }

    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "SwerveDriveFollowPathAction-" + pathname_ ;
        return ret ;
    }
}
