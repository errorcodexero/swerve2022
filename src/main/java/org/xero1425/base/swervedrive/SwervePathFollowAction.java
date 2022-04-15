package org.xero1425.base.swervedrive;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwervePathFollowAction extends SwerveDriveAction {
    private int index_;
    private String pathname_;
    private XeroPath path_;
    private double[] angles_;
    private double[] speeds_;
    private Number[] path_data_ ;

    private NetworkTableInstance inst_ ;
    private NetworkTable table_ ;

    private final static String PathKey = "XeroPath" ;

    public SwervePathFollowAction(SwerveDriveSubsystem drive, String path) {

        super(drive);

        pathname_ = path;
        path_ = null;

        angles_ = new double[4] ;
        speeds_ = new double[4] ;

        path_data_ = new Number[3] ;
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

        inst_ = NetworkTableInstance.getDefault() ;
        table_ = inst_.getTable(PathKey) ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        if (index_ < path_.getSize())
        {
            NetworkTableEntry entry ;

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

            System.out.println("path " + speeds_[0] + " " + speeds_[1] + " " + speeds_[2] + " " + speeds_[3]);

            getSubsystem().setTargets(angles_, speeds_);
            index_++ ;

            entry = table_.getEntry("path") ;
            path_data_[0] = (fl.getX() + fr.getX() + bl.getX() + br.getX()) / 4.0 ;
            path_data_[1] = (fl.getY() + fr.getY() + bl.getY() + br.getY()) / 4.0 ;
            path_data_[2] = fl.getHeading() ;
            entry.setNumberArray(path_data_) ;

            Pose2d pose = getSubsystem().getPose() ;
            entry = table_.getEntry("robot") ;
            path_data_[0] = pose.getX() ;
            path_data_[1] = pose.getY() ;
            path_data_[2] = pose.getRotation().getDegrees() ;
            entry.setNumberArray(path_data_) ;
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
}
