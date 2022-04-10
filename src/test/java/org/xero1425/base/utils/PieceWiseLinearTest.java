package org.xero1425.base.utils;

import java.util.ArrayList;
import java.util.List;

import org.junit.*;

import edu.wpi.first.math.geometry.Translation2d;

public class PieceWiseLinearTest {
    @Before
    public void init() {
    }

    @Test
    public void emptyListTest() {
        boolean except = false;
        List<Translation2d> points = new ArrayList<Translation2d>();
        PieceWiseLinear pwl = null ;

        try {
            pwl = new PieceWiseLinear(points);
        } catch (Exception ex) {
            except = true;
        }

        Assert.assertNull(pwl);
        Assert.assertEquals(true, except);
    }

    @Test
    public void oneSegTest() throws Exception {
        List<Translation2d> points = new ArrayList<Translation2d>();
        points.add(new Translation2d(0.0, 0.0));
        points.add(new Translation2d(10.0, 10.0));

        PieceWiseLinear pwl = new PieceWiseLinear(points);

        Assert.assertEquals(0.0, pwl.getValue(-10.0), 1e-6) ;
        Assert.assertEquals(10.0, pwl.getValue(20.0), 1e-6) ;
        Assert.assertEquals(5.0, pwl.getValue(5.0), 1e-6) ;
    }

    @Test
    public void twoSegTest() throws Exception {
        List<Translation2d> points = new ArrayList<Translation2d>();
        points.add(new Translation2d(0.0, 0.0));
        points.add(new Translation2d(10.0, 10.0));
        points.add(new Translation2d(20.0, 0.0));

        PieceWiseLinear pwl = new PieceWiseLinear(points);

        Assert.assertEquals(0.0, pwl.getValue(-10.0), 1e-6) ;
        Assert.assertEquals(0.0, pwl.getValue(30.0), 1e-6) ;
        Assert.assertEquals(5.0, pwl.getValue(5.0), 1e-6) ;
        Assert.assertEquals(5.0, pwl.getValue(15.0), 1e-6) ;
    }
} ;