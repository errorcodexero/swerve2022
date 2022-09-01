package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.swerve.xeroswerve.XeroSwerveDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.MissingPathException;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveHolonomicPathFollower extends SwerveDriveAction {

    private String pathname_ ;
    private HolonomicDriveController ctrl_ ;
    private XeroPath path_;
    private int index_ ;
    private Rotation2d end_rotation_ ;

    public SwerveHolonomicPathFollower(SwerveBaseSubsystem sub, String pathname, double endangle) {
        super(sub) ;

        pathname_ = pathname ;
        end_rotation_ = Rotation2d.fromDegrees(endangle) ;
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException, MissingPathException {
        double kp, ki, kd ;

        double maxv = getSubsystem().getSettingsValue("physical:max-angular-speed").getDouble() ;
        double maxa = getSubsystem().getSettingsValue("physical:max-angular-accel").getDouble() ;

        kp = getSubsystem().getSettingsValue("pid:xctrl:kp").getDouble() ;
        ki = getSubsystem().getSettingsValue("pid:xctrl:ki").getDouble() ;
        kd = getSubsystem().getSettingsValue("pid:xctrl:kd").getDouble() ;
        PIDController xctrl = new PIDController(kp, ki, kd) ;

        kp = getSubsystem().getSettingsValue("pid:yctrl:kp").getDouble() ;
        ki = getSubsystem().getSettingsValue("pid:yctrl:ki").getDouble() ;
        kd = getSubsystem().getSettingsValue("pid:yctrl:kd").getDouble() ;
        PIDController yctrl = new PIDController(kp, ki, kd) ;

        kp = getSubsystem().getSettingsValue("pid:rotation:kp").getDouble() ;
        ki = getSubsystem().getSettingsValue("pid:rotation:ki").getDouble() ;
        kd = getSubsystem().getSettingsValue("pid:rotation:kd").getDouble() ;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxv, maxa) ;
        ProfiledPIDController thetactrl = new ProfiledPIDController(kp, ki, kd, constraints) ;
        
        ctrl_ = new HolonomicDriveController(xctrl, yctrl, thetactrl) ;
        ctrl_.setEnabled(true);
        Pose2d done = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5)) ;
        ctrl_.setTolerance(done);

        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);

        Pose2d pose = getPoseFromPath(0) ;
        getSubsystem().resetOdometry(pose);

        index_ = 0 ;

        getSubsystem().startSwervePlot("SwerveHolonomicPathFollower") ;
        getSubsystem().startPathing();
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {

        if (index_ < path_.getSize())
        {
            Rotation2d rot = getSubsystem().getPose().getRotation() ;
            if (index_ > 200)
                rot = end_rotation_ ;
            Pose2d target = getPoseFromPath(index_);
            getSubsystem().setPathLocation(target);
            double velocity = getVelocityFromPath(index_) ;
            ChassisSpeeds speed = ctrl_.calculate(getSubsystem().getPose(), target, velocity, rot) ;
            getSubsystem().drive(speed) ;

            if (index_ != path_.getSize() - 1 || ctrl_.atReference())
                index_++ ;
        }

        if (index_ >= path_.getSize())
        {
            getSubsystem().endSwervePlot() ;
            getSubsystem().drive(new ChassisSpeeds()) ;
            getSubsystem().endPathing();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveHolonomicPathFollower " + pathname_ ;
    }

    private Pose2d getPoseFromPath(int index) {
        XeroPathSegment fl = path_.getSegment(XeroSwerveDriveSubsystem.FL, index) ;
        XeroPathSegment fr = path_.getSegment(XeroSwerveDriveSubsystem.FR, index) ;
        XeroPathSegment bl = path_.getSegment(XeroSwerveDriveSubsystem.BL, index) ;
        XeroPathSegment br = path_.getSegment(XeroSwerveDriveSubsystem.BR, index) ;

        double x = (fl.getX() + fr.getX() + bl.getX() + br.getX()) / 4.0 ;
        double y = (fl.getY() + fr.getY() + bl.getY() + br.getY()) / 4.0 ;
        double heading = fl.getHeading() ;

        return new Pose2d(x, y, Rotation2d.fromDegrees(heading)) ;
    }

    private double getVelocityFromPath(int index) {
        XeroPathSegment fl = path_.getSegment(XeroSwerveDriveSubsystem.FL, index) ;
        XeroPathSegment fr = path_.getSegment(XeroSwerveDriveSubsystem.FR, index) ;
        XeroPathSegment bl = path_.getSegment(XeroSwerveDriveSubsystem.BL, index) ;
        XeroPathSegment br = path_.getSegment(XeroSwerveDriveSubsystem.BR, index) ;

        double ret = (fl.getVelocity() + fr.getVelocity() + bl.getVelocity() + br.getVelocity())  / 4.0 ;
        return ret ;
    }
}
