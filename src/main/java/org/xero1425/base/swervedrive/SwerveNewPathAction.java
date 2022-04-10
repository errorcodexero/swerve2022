package org.xero1425.base.swervedrive;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.Subsystem.DisplayType;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;
import edu.wpi.first.math.geometry.Pose2d;

public class SwerveNewPathAction extends SwerveDriveAction {
    private String pathname_ ;
    private XeroPath path_ ;
    private int index_ ;
    private double [] angles_ ;
    private double [] speeds_ ;

    static private final int MainRobot = 0 ;

    public SwerveNewPathAction(SwerveDriveSubsystem sub, String pathname) {
        super(sub) ;

        pathname_ = pathname ;
        angles_ = new double[getSubsystem().getModuleCount()] ;
        speeds_ = new double[getSubsystem().getModuleCount()] ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        getSubsystem().startSwervePlot();

        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);
        index_ = 0 ;

        Pose2d initial = DriveBaseSubsystem.segmentToPose(path_.getSegment(MainRobot, 0)) ;
        getSubsystem().resetOdometry(initial) ;
    }

    @Override
    public void run() {
        if (index_ < path_.getSize())
        {
            var sub = getSubsystem() ;
            XeroPathSegment seg = path_.getSegment(MainRobot, index_) ;

            Pose2d pos = getSubsystem().getPose() ;
            Pose2d desired = DriveBaseSubsystem.segmentToPose(seg) ;
            Pose2d err = desired.relativeTo(pos) ;


            //
            // The desired angle is to get us back to the desired position
            // The desired velocity is the velocity of the path
            //
            double angle = Math.toDegrees(Math.atan2(err.getY(), err.getX())) ;

            for(int i = 0 ; i < getSubsystem().getModuleCount() ; i++)
            {
                angles_[i] = angle ;
                speeds_[i] = seg.getVelocity() ;
            }

            getSubsystem().setTargets(angles_, speeds_);

            sub.putDashboard("db-trk-t", DisplayType.Always, sub.getRobot().getTime()) ;
            sub.putDashboard("db-trk-x", DisplayType.Always, pos.getX()) ;
            sub.putDashboard("db-trk-y", DisplayType.Always, pos.getY()) ;
            sub.putDashboard("db-trk-a", DisplayType.Always, pos.getRotation().getDegrees()) ;

            sub.putDashboard("db-path-t", DisplayType.Always, sub.getRobot().getTime()) ;
            sub.putDashboard("db-path-x", DisplayType.Always, seg.getX()) ;
            sub.putDashboard("db-path-y", DisplayType.Always, seg.getY()) ;
            sub.putDashboard("db-path-a", DisplayType.Always, seg.getHeading()) ;
        }

        index_++ ;
        
        if (index_ == path_.getSize())
        {
            setDone() ;
            getSubsystem().endSwervePlot();
        }
    }

    @Override
    public void cancel() {
    }


    @Override
    public String toString(int ident) {
        return "SwerveNewPathAction" ;
    }
}
