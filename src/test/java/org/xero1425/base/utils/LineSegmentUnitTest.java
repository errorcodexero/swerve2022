package org.xero1425.base.utils;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Assert;

public class LineSegmentUnitTest {

    @Before
    public void init() {
    }

    @Test
    public void testLineSegmentLength() {
        LineSegment ls = new LineSegment(0, 0, 100, 0) ;
        Assert.assertEquals(100.0, ls.length(), 0.0001) ;

        ls = new LineSegment(1, 1, 10, 10) ;
        Assert.assertEquals(12.7279220613, ls.length(), 0.00001);
    }

    @Test
    public void testLineSegmentPointIntersectHorizontal() {
        LineSegment ls = new LineSegment(0, 0, 100, 0) ;
        Translation2d ret ;

        ret = ls.closest(new Translation2d(200, 8)) ;
        Assert.assertEquals(100, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(200, -8)) ;
        Assert.assertEquals(100, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(-200, 8)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(-200, -8)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(25, 8)) ;
        Assert.assertEquals(25, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(25, -8)) ;
        Assert.assertEquals(25, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);
    }

    @Test
    public void testLineSegmentPointIntersectVertical() {
        LineSegment ls = new LineSegment(0, 0, 0, -100) ;
        Translation2d ret ;

        ret = ls.closest(new Translation2d(8, 100)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(-8, 100)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(0, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(8, -200)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(-100, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(-8, -200)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(-100, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(8, -75)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(-75, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(-8, -75)) ;
        Assert.assertEquals(0, ret.getX(), 0.000001);
        Assert.assertEquals(-75, ret.getY(), 0.000001);
    }

    @Test
    public void testLineSegmentPointIntersectGeneral() {
        LineSegment ls = new LineSegment(0, 0, 100, 100) ;
        Translation2d ret ;

        ret = ls.closest(new Translation2d(100, 0)) ;
        Assert.assertEquals(50, ret.getX(), 0.000001);
        Assert.assertEquals(50, ret.getY(), 0.000001);

        ret = ls.closest(new Translation2d(0, 100)) ;
        Assert.assertEquals(50, ret.getX(), 0.000001);
        Assert.assertEquals(50, ret.getY(), 0.000001);
    }
};
