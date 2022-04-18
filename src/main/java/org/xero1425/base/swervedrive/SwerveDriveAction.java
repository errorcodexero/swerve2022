package org.xero1425.base.swervedrive;

import org.xero1425.base.actions.Action;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SwerveDriveAction extends Action {
    private SwerveDriveSubsystem swerverive_ ;
    private double circum_ ;

    /// \brief Create the object holding a reference to the subsystem
    /// \param drive the tankdrive subsystem
    public SwerveDriveAction(SwerveDriveSubsystem drive) {
        super(drive.getRobot().getMessageLogger()) ;
        swerverive_ = drive ;

        //
        // We approximate the circumference of the robot by averaging the length and width and applying
        // the CIRCUM = 2 * PI * R.  Calculating the circumference of an ellipse is more computational intensive
        // and really does not get us that much.
        //
        circum_ = (swerverive_.getLength() + swerverive_.getWidth()) * Math.PI / 2.0 ;
    }

    /// \brief return the tank drive subsystem
    /// returns the tank drive subsystem
    public SwerveDriveSubsystem getSubsystem() {
        return swerverive_ ;
    }

    protected Pose2d combinePose(Pose2d fl, Pose2d fr, Pose2d bl, Pose2d br) {
        double x = (fl.getX() + fr.getX() + bl.getX() + br.getX()) / 4.0 ;
        double y = (fl.getY() + fr.getY() + bl.getY() + br.getY()) / 4.0 ;
        double h = (fl.getRotation().getDegrees() + fr.getRotation().getDegrees()  + bl.getRotation().getDegrees()  + br.getRotation().getDegrees()) / 4.0 ;

        return new Pose2d(x, y, Rotation2d.fromDegrees(h)) ;
    }

    protected Translation2d rotateVector(Translation2d vec, double angle) {
        double rads = angle / 180.0 * Math.PI ;
        return new Translation2d(vec.getX() * Math.cos(rads) - vec.getY() * Math.sin(rads), vec.getY() * Math.cos(rads) + vec.getX() * Math.sin(rads)) ;
    }

    protected Translation2d createRotVector(int which, double rot) {
        //
        // The rot value is in degress per second, we need to transform this to a
        // linear speed for the wheel
        //
        double linear = rot / 360.0 * circum_ / 2 ;
        double angle = 0.0 ;
        double phi = getSubsystem().getPHI() ;

        switch(which) {
            case SwerveDriveSubsystem.FL:
                angle = 180 - phi ;
                break ;
            case SwerveDriveSubsystem.FR:
                angle = phi ;
                break ;
            case SwerveDriveSubsystem.BL:
                angle = -180 + phi ;
                break ;
            case SwerveDriveSubsystem.BR:
                angle = -phi ;
                break ;                                                
        }

        angle = angle / 180.0 * Math.PI ;
        return new Translation2d(Math.cos(angle) * linear, Math.sin(angle) * linear) ;
    }

    protected Translation2d addVectors(Translation2d v1, Translation2d v2) {
        return new Translation2d(v1.getX() + v2.getX(), v1.getY() + v2.getY()) ;
    }
}
