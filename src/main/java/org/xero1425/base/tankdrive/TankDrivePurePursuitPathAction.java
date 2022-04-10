package org.xero1425.base.tankdrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.utils.LineSegment;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.MissingPathException;
import org.xero1425.misc.PIDCtrl;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/// \file

/// \brief This class implements the pure pursuit path following algorithm
/// Note, this code is not working yet. It is experimental and should be used to
/// play with the algorithm only.
public class TankDrivePurePursuitPathAction extends TankDriveAction {
    private double start_time_ ;
    private String path_name_;
    private double look_ahead_distance_;
    private XeroPath path_;
    private PIDCtrl left_pid_;
    private PIDCtrl right_pid_;
    private int plot_id_ ;
    private Double[] plot_data_ ;
    private int cycle_ ;
    private double current_vel_ ;
    private double max_accel_ ;

    // The values to plot when using this action
    static private final String[] plot_columns_ = {             
        "time", 
        "lavel", "ltvel", "lout", 
        "ravel", "rtvel", "rout", 
    } ;

    // The data to extract from a path
    private final int MainRobot = 0;

    /// \brief Create the action to follow a path using the pure pursuit algorithm
    /// \param drive the tank drive subsystem
    /// \param path the name of the path to follow, it must be present in the XeroPathManager
    /// \param reverse if true follow the path in reverse
    public TankDrivePurePursuitPathAction(TankDriveSubsystem drive, String path, boolean reverse)
            throws MissingPathException, MissingParameterException, BadParameterTypeException {
        super(drive);

        path_name_ = path;
        look_ahead_distance_ = drive.getSettingsValue("tankdrive:purepursuit:lookahead").getDouble() ;
        max_accel_ = drive.getSettingsValue("tankdrive:purepursuit:maxaccel").getDouble() ;       

        path_ = getSubsystem().getRobot().getPathManager().getPath(path) ;
        plot_id_ = drive.initPlot(toString(0)) ;
        plot_data_= new Double[plot_columns_.length] ;

        ISettingsSupplier settings = getSubsystem().getRobot().getSettingsSupplier() ;
        left_pid_ = new PIDCtrl(settings, "tankdrive:purepursuit:left", false) ;
        right_pid_ = new PIDCtrl(settings, "tankdrive:purepursuit:right", false) ;
    }

    /// \brief Start the path following action
    /// Called once to initializa the action to follow the path.  This method sets the initial
    /// robot pose, stores the initial time, and starts plotting.
    @Override
    public void start() {

        start_time_ = getSubsystem().getRobot().getTime() ;

        XeroPathSegment seg = path_.getSegment(MainRobot, 0) ;
        Pose2d start = new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
        getSubsystem().setPose(start);

        getSubsystem().startPlot(plot_id_, plot_columns_);

        getSubsystem().setRecording(true);

        cycle_ = 0 ;
    }

    /// \brief Run the path following action
    /// Called each robot loop to adjust the robot's velocity to follow the path
    @Override
    public void run() {
        //
        // Get the tank drive subsystem
        //
        TankDriveSubsystem sub = getSubsystem() ;

        //
        // Get the robot
        //
        XeroRobot robot = sub.getRobot() ;

        //
        // Get the robots current position
        //
        Pose2d current = getSubsystem().getPose() ;

        //
        // Find the point on the path that is closest in distance to the robots
        // current position
        //
        PathPoint closest = findClosestPoint(current) ;

        //
        // Find point at the look ahead distance from here to the robot
        //
        LookAheadPoint look = findLookAheadPoint(closest) ;

        if (!look.atEnd())
        {
            //
            // Find the curved arc we need to drive from the current
            // position to the look ahead position
            //
            double curvature = findDrivingCurvature(current, look.getPose()) ;

            current_vel_ = current_vel_ + max_accel_ * robot.getDeltaTime() ;
                
            double width = getSubsystem().getWidth() / getSubsystem().getScrub() ;

            //
            // Compute the left and right drive velocities
            //
            TankDriveVelocities vel = inverseKinematics(curvature, current_vel_, width) ;

            double left_out = left_pid_.getOutput(vel.getLeft(), sub.getLeftVelocity(), sub.getRobot().getDeltaTime()) ;
            double right_out = right_pid_.getOutput(vel.getRight(), sub.getRightVelocity(), sub.getRobot().getDeltaTime());

            double delta = look.getPose().getX() - current.getX() ;

            MessageLogger logger = sub.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, sub.getLoggerID()) ;
            logger.add("purepursuit:") ;
            logger.add("cycle", cycle_) ;
            logger.add("rx", current.getX()) ;
            logger.add("ry", current.getY()) ;
            logger.add("ra", current.getRotation().getDegrees()) ;
            logger.add("lx", look.getPose().getX()) ;
            logger.add("ly", look.getPose().getY()) ;
            logger.add("la", look.getPose().getRotation().getDegrees()) ;
            logger.add("delta", delta) ;
            logger.add("curv", curvature) ;
            logger.add("velocity", current_vel_) ;
            logger.add("left", vel.getLeft()) ;
            logger.add("right", vel.getRight()) ;
            logger.add("leftpower", left_out) ;
            logger.add("rightpower", right_out) ;
            logger.endMessage();

            sub.setPower(left_out, right_out) ;

            plot_data_[0] = robot.getTime() - start_time_ ;
            plot_data_[1] = sub.getLeftVelocity() ;
            plot_data_[2] = vel.getLeft() ;
            plot_data_[3] = left_out ;
            plot_data_[4] = sub.getRightVelocity() ;
            plot_data_[5] = vel.getRight() ;
            plot_data_[6] = right_out ;

            cycle_++ ;
        }
        else
        {
            getSubsystem().setRecording(false);
            setDone() ;
        }

    }

    /// \brief Cancel the path follower action
    /// This marks the action done and sets the motor power to zero
    @Override
    public void cancel() {
        super.cancel() ;

        getSubsystem().setPower(0.0, 0.0) ;
    }

    /// \brief Returns a human readable string for the action
    /// \returns a human readable string for the action
    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "TankDrivePurePursuitPathAction-" + path_name_ ;
        return ret ;
    }

    private TankDriveVelocities inverseKinematics(double curvature, double velocity, double width) {

        double l = velocity * (2 + curvature * width) / 2.0 ;
        double r = velocity * (2 - curvature * width) / 2.0 ;

        return new TankDriveVelocities(l, r) ;
    }

    private double findDrivingCurvature(Pose2d current, Pose2d target) {
        double a, b, c ;

        a = -Math.tan(current.getRotation().getRadians()) ;
        b = 1.0 ;
        c = Math.tan(current.getRotation().getRadians()) * current.getX() - current.getY() ;

        double x = Math.abs(a * target.getX() + b * target.getY() + c) / Math.sqrt(a * a + b * b) ;

        double ang = current.getRotation().getRadians() ;
        double z = Math.sin(ang) * (target.getX() - current.getX()) - Math.cos(ang) * (target.getY() - current.getY()) ;
        double curv = 2 * x / (look_ahead_distance_ * look_ahead_distance_)  * Math.signum(z) ;

        return curv ;
    }

    private PathPoint findClosestPoint(Pose2d pos) {
        double dist = Double.MAX_VALUE ;
        int which = -1 ;
        double pcnt = 0.0 ;

        Translation2d p1 = null, p2 = null ;

        for(int i = 0 ; i < path_.getSize() - 1; i++) {
            XeroPathSegment seg0 = path_.getSegment(MainRobot, i) ;
            XeroPathSegment seg1 = path_.getSegment(MainRobot, i + 1) ;

            LineSegment ls = new LineSegment(seg0.getX(), seg0.getY(), seg1.getX(), seg1.getY()) ;
            Translation2d closest = ls.closest(pos.getTranslation()) ;
            double clpcnt = ls.dotProd(pos.getTranslation()) / ls.length() ;

            double ptdist = closest.getDistance(pos.getTranslation()) ;
            if (ptdist < dist)
            {
                which = i ;
                dist = ptdist ;
                pcnt = clpcnt ;
                p1 = new Translation2d(seg0.getX(), seg0.getY()) ;
                p2 = new Translation2d(seg1.getX(), seg1.getY()) ;
            }
        }

        double x = p1.getX() + (p2.getX() - p2.getX()) * pcnt ;
        double y = p1.getY() + (p2.getY() - p2.getY()) * pcnt ;
        return new PathPoint(which, pcnt, new Translation2d(x, y)) ;
    }

    private Pose2d interpolate(Pose2d p1, Pose2d p2, double param) {
        double x = p1.getX() + (p2.getX() - p1.getX()) * param ;
        double y = p1.getY() + (p2.getY() - p1.getY()) * param ;
        double heading = p1.getRotation().getRadians() + (p2.getRotation().getRadians() - p1.getRotation().getRadians()) * param ;

        return new Pose2d(x, y, new Rotation2d(heading)) ;
    }

    private Pose2d segmentToPose(XeroPathSegment seg) {
        return new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
    }

    private LookAheadPoint findLookAheadPoint(PathPoint pt) {
        LookAheadPoint ret = null ;

        double remaining = look_ahead_distance_ ;

        for(int i = pt.which() ; i < path_.getSize() - 1 ; i++)
        {
            XeroPathSegment seg0 = path_.getSegment(MainRobot, i);
            double prev = 0.0 ;
            if (i != 0)
                prev = path_.getSegment(MainRobot, i - 1).getPosition() ;

            double segsize = seg0.getPosition() - prev ;

            if (remaining < segsize)
            {
                //
                // The look ahead point is in this segment
                //

                XeroPathSegment seg1 = path_.getSegment(MainRobot, i + 1) ;

                // Calculate how far is the look ahead point between this segment and the next
                double pcnt = 1 - remaining / segsize ;
                ret = new LookAheadPoint(interpolate(segmentToPose(seg0), segmentToPose(seg1), pcnt), false) ;
                break ;
            }

            remaining -= segsize ;
        }

        if (ret == null)
        {
            //
            // The look ahead point exceeds the path
            //
            XeroPathSegment seg = path_.getSegment(MainRobot, path_.getSize() - 1) ;
            Pose2d endpt = segmentToPose(seg) ;

            boolean atend = false ;

            double dist = endpt.getTranslation().getDistance(pt.loc()) ;
            if (dist < 0.1)
                atend = true ;
            ret = new LookAheadPoint(endpt, atend) ;
        }

        return ret ;
    }
}
