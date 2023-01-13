package frc.robot.Utils;

public class PID {
    
    public double p;
    public double i;
    public double d;

    /** the Integral Accumulator */
    public double iv;
    /** the last process variable used for calculating derivitive term */
    public double lpv;

    private long lastTime = 0;

    public PID(double P, double I, double D) {
        p = P;
        i = I;
        d = D;
        lastTime = System.currentTimeMillis();
    }

    public void reset() {
        iv = 0;
        lpv = 0;
    }

    public double calc(double pv) {
        long t = System.currentTimeMillis();
        double tv = (lastTime - t)/1000;
        if (tv > 0.5) tv = 0;
        iv += pv * tv;
        lastTime = t;
        // pv * p + acc * i - DeltaPV * ts * d
        double res = (pv * p) - (iv * i) + (((pv - lpv) * tv) * d);
        lpv = pv;
        return res;
    }
}
