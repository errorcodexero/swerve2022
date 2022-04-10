package org.xero1425.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/// \file

/// \brief This class tracks the position of the drivebase.
public class PositionTracker {

    /// The current pose of the robot (X position, Y position, and heading)
    private Pose2d pose_ ;

    /// The width of the robot
    private double width_ ;

    // The scrub for the robot
    private double scrub_ ;

    /// \brief Create a new tank drive position tracker
    /// \param width the width of the robot in inches
    /// \param scrub the scrub factor for the robot
    public PositionTracker(double width, double scrub) {
        width_ = width ;
        scrub_ = scrub ;
        pose_ = new Pose2d(0.0, 0.0, new Rotation2d(0.0)) ;
    }

    /// \brief Retrunt he current pose of the robot
    /// \returns the current pose of the robot
    public Pose2d getPose() {
        return pose_ ;
    }

    /// \brief Set the current pose of the robot
    /// \param pose the pose for the robot
    public void setPose(Pose2d pose) {
        pose_ = pose ;
    }

    /// \brief Return the width of the robot
    /// \returns the width of the robot
    public double getWidth() {
        return width_ ;
    }

    /// \brief Return the scrub factor for the robot
    /// \returns the scrub factor for the robot
    public double getScrub() {
        return scrub_ ;
    }

    /// \brief Update the current robot position
    /// \param dleft the distance the left wheel has traveled in the last robot loop
    /// \param dright the distance the right wheel has traveled in the last robot loop
    /// \param dangle the current heading of the robot
    public void updatePosition(double dleft, double dright, double dangle) {

        double xpos, ypos ;
        double angle = dangle * Math.PI / 180.0 ;

        if (Math.abs(dleft - dright) < 1e-6) {
            xpos = pose_.getX() +  dleft * Math.cos(angle) ;
            ypos = pose_.getY() + dright * Math.sin(angle) ;
        }
        else {
            double r = width_ * (dleft + dright) / (2 * (dright - dleft)) ;
            double wd = (dright - dleft) / width_ ;
            xpos = pose_.getX() + r * Math.sin(wd + angle) - r * Math.sin(angle) ;
            ypos = pose_.getY() - r * Math.cos(wd + angle) + r * Math.cos(angle) ;
        }

        pose_ = new Pose2d(xpos, ypos, Rotation2d.fromDegrees(angle)) ;
    }
}
