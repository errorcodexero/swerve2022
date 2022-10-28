package org.xero1425.misc;
import org.junit.*;

public class MinMaxDataTest {
    @Before
    public void init() {
    }

    @Test
    public void fullTest() {
        MinMaxData data = new MinMaxData(8) ;
        data.addData(10.0);
        data.addData(2.0);
        data.addData(13.0);
        data.addData(42.0);
        data.addData(11.0);
        data.addData(91.0);
        data.addData(21.0);
        data.addData(188.0);

        Assert.assertEquals(188, data.getMax(), 1e-6);
        Assert.assertEquals(2.0, data.getMin(), 1e-6);
    }

    @Test
    public void fullDuplicatedTest() {
        MinMaxData data = new MinMaxData(8) ;
        data.addData(10.0);
        data.addData(2.0);
        data.addData(188.0);
        data.addData(42.0);
        data.addData(11.0);
        data.addData(2.0);
        data.addData(21.0);
        data.addData(188.0);

        Assert.assertEquals(188, data.getMax(), 1e-6);
        Assert.assertEquals(2.0, data.getMin(), 1e-6);
    }

    @Test
    public void partial() {
        MinMaxData data = new MinMaxData(8) ;
        data.addData(10.0);
        data.addData(2.0);
        data.addData(21.0);
        data.addData(188.0);

        Assert.assertEquals(188, data.getMax(), 1e-6);
        Assert.assertEquals(2.0, data.getMin(), 1e-6);
    }

    @Test
    public void partialDuplicated() {
        MinMaxData data = new MinMaxData(8) ;
        data.addData(10.0);
        data.addData(2.0);
        data.addData(21.0);
        data.addData(188.0);
        data.addData(2.0);
        data.addData(21.0);
        data.addData(188.0);

        Assert.assertEquals(188, data.getMax(), 1e-6);
        Assert.assertEquals(2.0, data.getMin(), 1e-6);
    }
}
