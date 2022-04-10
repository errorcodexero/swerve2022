package org.xero1425.base.tankdrive;

/// \file

/// \brief This class stores velocities for the left and right wheels of a tank drive
public class TankDriveVelocities {
    private double left_ ;
    private double right_ ;

    /// \brief This constructor creates an object with the given left and right velocities
    public TankDriveVelocities(double l, double r) {
        left_ = l ;
        right_ = r ;
    }

    /// \brief returns the left wheel velocity for the tank drive
    /// \returns the left wheel velocity for the tank drive
    public double getLeft() {
        return left_ ;
    }

    /// \brief returns the right wheel velocity for the tank drive
    /// \returns the right wheel velocity for the tank drive
    public double getRight() {
        return right_ ;
    }
}
