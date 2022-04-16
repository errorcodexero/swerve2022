package org.xero1425.base.swervedrive;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwervePathFollowAction extends SwerveDriveAction {
    private int index_;
    private String pathname_;
    private XeroPath path_;
    private double[] angles_;
    private double[] speeds_;

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

        index_ = 0;
        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);

        XeroPathSegment fl = path_.getSegment(SwerveDriveSubsystem.FL, 0) ;
        XeroPathSegment fr = path_.getSegment(SwerveDriveSubsystem.FR, 0) ;
        XeroPathSegment bl = path_.getSegment(SwerveDriveSubsystem.BL, 0) ;
        XeroPathSegment br = path_.getSegment(SwerveDriveSubsystem.BR, 0) ;

        double x = (fl.getX() + fr.getX() + bl.getX() + br.getX()) / 4.0 ;
        double y = (fl.getY() + fr.getY() + bl.getY() + br.getY()) / 4.0 ;
        double heading = fl.getHeading() ;

        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(heading)) ;
        getSubsystem().resetOdometry(pose);

        getSubsystem().startSwervePlot("SwervePathFollowAction") ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        if (index_ < path_.getSize())
        {
            XeroPathSegment fl = path_.getSegment(SwerveDriveSubsystem.FL, index_) ;
            XeroPathSegment fr = path_.getSegment(SwerveDriveSubsystem.FR, index_) ;
            XeroPathSegment bl = path_.getSegment(SwerveDriveSubsystem.BL, index_) ;
            XeroPathSegment br = path_.getSegment(SwerveDriveSubsystem.BR, index_) ;

            angles_[SwerveDriveSubsystem.FL] = fl.getHeading() ;
            angles_[SwerveDriveSubsystem.FR] = fr.getHeading() ;
            angles_[SwerveDriveSubsystem.BL] = bl.getHeading() ;
            angles_[SwerveDriveSubsystem.BR] = br.getHeading() ;

            speeds_[SwerveDriveSubsystem.FL] = fl.getVelocity() ;
            speeds_[SwerveDriveSubsystem.FR] = fr.getVelocity() ;
            speeds_[SwerveDriveSubsystem.BL] = bl.getVelocity() ;
            speeds_[SwerveDriveSubsystem.BR] = br.getVelocity() ;

            outputPath(getPoseFromPath(index_), fl.getRotation()) ;

            getSubsystem().setTargets(angles_, speeds_);
            index_++ ;
        }

        if (index_ == path_.getSize())
        {
            getSubsystem().stop();
            setDone() ;
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

    private Pose2d getPoseFromPath(int index) {
        XeroPathSegment fl = path_.getSegment(SwerveDriveSubsystem.FL, index) ;
        XeroPathSegment fr = path_.getSegment(SwerveDriveSubsystem.FR, index) ;
        XeroPathSegment bl = path_.getSegment(SwerveDriveSubsystem.BL, index) ;
        XeroPathSegment br = path_.getSegment(SwerveDriveSubsystem.BR, index) ;

        double x = (fl.getX() + fr.getX() + bl.getX() + br.getX()) / 4.0 ;
        double y = (fl.getY() + fr.getY() + bl.getY() + br.getY()) / 4.0 ;
        double heading = fl.getHeading() ;

        return new Pose2d(x, y, Rotation2d.fromDegrees(heading)) ;
    }    
}
