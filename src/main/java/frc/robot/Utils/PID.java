package frc.robot.Utils;

public class PID {
    
    public double p;
    public double i;
    public double d;

    public double iv;
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
        long tv = (lastTime - t)/1000;
        if (tv > 500) tv = 0;
        iv += pv * (lastTime - t);
        lastTime = t;
        double res = (pv * p) - (iv * i) + (((pv - lpv) * tv) * d);
        lpv = pv;
        return res;
    }
}
