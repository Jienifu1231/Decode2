package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

public class Convolution {

    public List<Double> values = new ArrayList<>();
    public int size;
    public double sigma = 2.5;

    double sum = 0;
    double average = 0;
    double power_sum = 0;
    double std = 0;
    double median = 0;
    int n = 0;

    public boolean steady = false;

    public Convolution(int size){
        this.size = size;
    }

    public Convolution(int size, double sigma){
        this.size = size;
        this.sigma = sigma;
    }

    public double getAverage(){ return average; }
    public double getMedian(){ return median; }
    public double getSTD(){ return std; }
    public boolean full(){ return values.size() == size; }
    public boolean steady(){ return steady; }

    public void update(double new_value){
        if(values.size() < size){
            values.add(new_value);
        } else /*if (Math.abs(new_value - average) < sigma * std)*/{
            values.remove(0);
            values.add(new_value);
        }

        n = values.size();
        sum = 0;
        power_sum = 0;

        values.sort(null);
        if(n % 2 == 1){ median = values.get((n + 1)/2); }
        else { median = values.get(n/2); }

        for(int i = 0; i < n; i++){ sum += values.get(i); }
        average = sum / n;

        for(int i = 0; i < n; i++){ power_sum += Math.pow(values.get(i) - average, 2); }
        std = Math.sqrt(power_sum / n);

        steady = this.full();
        for(int i = 0; i < n; i++){ if(Math.abs(values.get(i) - average) > 2.5*std){ steady = false; } }
    }

}
