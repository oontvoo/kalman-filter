/**
 * Implementation of the Kalman Filter
 * Copyright (C) 2012  Vy Nguyen
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 * 
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

package filter;

import Jama.Matrix;
import java.io.*;
import java.util.*;
import filter.RunFilter.Point;
import impl.KalmanFilter;
import impl.KalmanFilter1;

/**
 *
 * @author Vy Nguyen
 */
public class RunFilter2
{
    private static List<Point> createIdealValues(int count, double dt,
                                                 double ax0, double ay0,
                                                 double vx0, double vy0,
                                                 double x0, double y0)
    {
        // x  = x0 + v * dt
        // v = v0 + a * dt
        
        List<Point> ret = new ArrayList<Point>();
        ret.add(new Point(x0, y0, 0, vx0, vy0, ax0, ay0)); // first point
        --count;
        
        double t = dt;
        for (int n = 0; n < count; ++n)
        {
            double vx = vx0 + ax0 * t;
            double x = x0 + vx0 * t + 0.5 * ay0 * t * t;
            
            double vy = vy0 + ay0 * t;
            double y = y0 + vy0 * t + 0.5 * ay0 * t * t;
            
            ret.add(new Point(x, y, t, vx, vy, ax0, ay0));
            t += dt;
        }
        return ret;
    }
    
    private static void writeToFile(String fileName, List<Point> toWrite) throws IOException
    {
        StringBuilder bd = new StringBuilder();
        
        for (Point p : toWrite)
            bd.append(p.x).append("     ").append(p.y).append(System.getProperty("line.separator"));
        
        FileWriter fstream = new FileWriter(fileName);
        BufferedWriter out = new BufferedWriter(fstream);
        out.write(bd.toString());
        out.close();
    }
    
    private static double parseArg(String args[])
    {
        if (args.length != 1)
        {
            System.out.println("default noise: " + DEFAULT_NOISE);
            return DEFAULT_NOISE;
        }
        return Double.parseDouble(args[0]);
    }
    
    private static List<Point> readSensorData(double dt, double ax)
    {
        try
        {
            Scanner input = new Scanner(new File(SENSOR));
            double t = 0;
            List<Point> ret = new ArrayList<Point>();
            
            while(input.hasNext())
            {
                double x = Double.parseDouble(input.next());
                double y = Double.parseDouble(input.next());
                ret.add(new Point(x, y, t, 0, 0, ax, ax));
                t += dt;
            }
            return ret;
        }
        catch (FileNotFoundException ex)
        {
            System.out.println("File not found: " + SENSOR);
            System.exit(2);
        }
        return null;
    }
    
    public static void main (String args[]) throws IOException
    {
        double ax = 2.5;
        double ay = 1.256;
        double dt = 0.8;

        System.out.println("computing the ideal values");
        List<Point> idealData = createIdealValues(9, dt,
                                          ax, ay ,
                                          0, 0,
                                          0, 0);
        writeToFile(IDEAL, idealData);

        System.out.println("reading sensor data");
        List<Point> sensor = readSensorData(dt, ax);
        double noise = parseArg(args);

        System.out.println("filtering sensor data");
        writeToFile(CORRECTED, filter(sensor, noise));
    }
     /**
     * 
     * @param inputs
     * @param sensorNoise
     * @return
     */
    public static List<Point> filter(List<Point> inputs, 
                                               double sensorNoise)
    {
        // sort the points in time order
        Collections.sort(inputs);

        // compute the velocity
        List<Point> updated = computeVAndA(inputs);
  
        // get the standard deviation of acceleration
        double sd = getSd(updated);

        List<Point> output = new ArrayList<Point>(inputs.size());
        output.add(inputs.get(0)); // first point is the same

        Matrix P0 = getP0();
        Matrix H = getH();
        Matrix R = createR(sensorNoise);
 
        KalmanFilter filterX = new KalmanFilter1(createXx(updated.get(0)),
                                                 P0,
                                                 H,
                                                 R
                                                 );
             
        KalmanFilter filterY = new KalmanFilter1(createXy(updated.get(0)),
                                                 P0,
                                                 H,
                                                 R
                                                 );
        for (int n = 1; n < updated.size(); ++n)
        {
            Point cur = updated.get(n);
            Matrix A = createA(cur);
            Matrix G = createG(cur);
            Matrix Gx = G.times(cur.ax);
            Matrix Gy = G.times(cur.ay);
            Matrix Q = getQ(G, sd);
            
            Matrix correctedX = filterX.timeUpdate(A, Q, Gx).messurementUpdate(createXx(cur));
            Matrix correctedY = filterY.timeUpdate(A, Q, Gy).messurementUpdate(createXy(cur));

            output.add(decodeXY(correctedX, correctedY, inputs.get(n)));
        }

        return output;
    }
    private static double getSd(List<Point> p)
    {
        // for now, the accelerations are const
        return 0;
    }

    /**
     * TODO: correctly decode the state matrix into a point.
     * (that is, retrieve a and v)
     * 
     * @param X the corrected state matrix
     * @return the corrected value of (x,y,t)
     */
    private static Point decodeXY(Matrix X, Matrix Y, Point input)
    {
        return new Point(X.get(0, 0) / 10, // x
                         Y.get(0, 0) / 10, // y
                         X.get(1, 0) / 10, // vx
                         Y.get(1, 0) / 10, // vy
                         input.ax, // unchanged
                         input.ay, // unchanged
                         input.t // unchanged
                );
    }
    
    /**
     * TODO: correctly encode the point (velocity, acceleration, etc.) into a matrix.
     * 
     * Vector x = {x, vx , ax, ay}
     * 
     * @param points 
     * @param currentIndex index of the point the filter is currently looking at
     * @return the input vector X
     */
    private static Matrix createXx(Point current)
    {
        Matrix x = new Matrix(2, 1);        

        x.set(0, 0, current.x);
        x.set(1, 0, current.vx);

        return x;
    }

    private static Matrix createXy(Point cur)
    {
        Matrix y = new Matrix(2, 1);
        
        y.set(0, 0, cur.y);
        y.set(1, 0, cur.vy);
        
        return y;
    }
    /**
     * Compute the instant velocity and acceleration of each point
     * <b>This method makes the following assumptions:</b>
     *      1. Acceleration of a Point object is constant and was properly set in the constructor.
     *      2. The velocity of the first point is either 0 or has already been correctly calculated.
     *    
     * TODO: take variable acceleration into account.
     * 
     * @param points
     * @return list of point with correct v and a 
     */
    private static List<Point> computeVAndA(List<Point> points)
    {
        int size = points.size();
        double vx_0 = points.get(0).vx;
        double vy_0 = points.get(0).vy;
        double ax_0 = points.get(0).ax;
        double ay_0 = points.get(0).ay;   
        
        List<Point> result = new ArrayList<Point> (size);
        result.add(points.get(0));
        
        for (int i = 1; i < size; ++i)
        {
            double dt = points.get(i).t - points.get(0).t;
            double vx = vx_0 + ax_0 * dt;
            double vy = vy_0 + ay_0 * dt;
            
            result.add(new Point(points.get(i).x,
                                 points.get(i).y,
                                 dt,
                                 vx,
                                 vy,
                                 points.get(i).ax,
                                 points.get(i).ay
                        ));
        }
        
        return result;
    }
    
    /**
     * TODO:
     * Same as createX(...)'s
     * 
     * @param inputs
     * @param currentIndex
     * @return the state matrix A based on the the current and past position
     */
    private static Matrix createA(Point current)
    {
        Matrix A = Matrix.identity(2, 2);
        A.set(0, 1, current.t);
        return A;
    }
    
    /**
     * 
     * @param error the sensor's error
     * @return the measurement noise covariance matrix
     */
    private static Matrix createR (double error)
    {
        Matrix R = new Matrix(1,1);
        R.set(0, 0, error);
        return R;
    }
    /**
     * 
     * @return the initial P matrix, which contains all zeroes 
     */
    private static Matrix getP0()
    {
        double m2x2[][] = new double[VARS_COUNT][VARS_COUNT];
        
        for (int n = 0; n < VARS_COUNT; ++n)
            Arrays.fill(m2x2[n], 0);

        return Matrix.constructWithCopy(m2x2);
    }

    /**
     * TODO:
     * correctly determine this
     * 
     * @return the process noise covariance matrix
     */
    private static Matrix getQ(Matrix G, double sd)
    {
        return G.times(G.transpose()).times(sd * sd);
    }

    /**
     * 
     * @return the [1 by n] observation matrix: [1 0 0 0 .... 0 ] 
     */
    private static Matrix getH()
    {
        Matrix H = new Matrix(1,2);
        H.set(0, 0, 1);
        return H;
    }
    
    private static Matrix createG(Point p)
    {
        Matrix ret = new Matrix(2, 1);
        ret.set(0, 0, p.t * p.t / 2);
        ret.set(1, 0, p.t);
        
        return ret;
    }

    private static final String IDEAL = "C:\\Users\\Sensor Research\\data\\ideal.txt";
    private static final String SENSOR = "C:\\Users\\\\Sensor Research\\data\\sensor.txt";
    private static final String CORRECTED = "C:\\Users\\Sensor Research\\data\\corrected.txt";

    private static final double DEFAULT_NOISE = 0.5;
    private static final int VARS_COUNT = 2;
}
