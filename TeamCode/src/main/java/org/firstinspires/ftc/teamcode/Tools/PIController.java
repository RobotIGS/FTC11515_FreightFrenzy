package org.firstinspires.ftc.teamcode.Tools;

public class PIController {
    private double integral = 0;

    private double p;
    private double i;

    public double out = 0;

    public PIController(double p, double i) {
        this.p = p;
        this.i = 0;
    }

    public void step(double error) {
        integral += i * error;
        out = integral + (p * error);
    }
}
