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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import impl.KalmanFilter2;

/**
 *
 * @author  Vy Nguyen
 */
public class RunFilter 
{
    /**
     * Represents a point
     */
    public static class Point implements Comparable<Point>
    {
        final double x; // x coord
        final double y; // y coord
        final double t; // time coord
        final double vx; // instant x-velocity
        final double vy; // instant y-velocity
        final double ax; // instant x-acceleration
        final double ay; // instant y-acceleration
        
        /**
         * 
         * @param x
         * @param y
         * @param t
         * @param vx
         * @param vy
         * @param ax
         * @param ay
         */
        public Point(double x, double y, double t, double vx, double vy, double ax, double ay)
        {
            this.x = x;
            this.y = y;
            this.t = t;
            this.vx = vx;
            this.vy = vy;
            this.ax = ax;
            this.ay = ay;
        }
        
        public String toString()
        {
            return "(" + x + ", "
                       + y + ", "
                       + vx + ", "
                       + vy + ", "
                       + ax + ", "
                       + ay + ", "
                       + t + ")";   
        }
        @Override
        public int compareTo(Point o)
        {
            return Double.compare(t, o.t);
        }
    }
    
    /**
     * 
     * @param inputs
     * @param sensorNoise
     * @return
     */
    public static List<Point> filter(List<Point> inputs, double sensorNoise)
    {
        // sort the points in time order
        Collections.sort(inputs);

        List<Point> output = new ArrayList<Point>(inputs.size());
        
        //(Matrix X, Matrix P, Matrix Q, Matrix H, Matrix A, Matrix R)
        KalmanFilter2 filter = new KalmanFilter2(createX(inputs, 0),
                                               getP0(),
                                               getQ(),
                                               getH(),
                                               createA(inputs, 0),
                                               createR(sensorNoise),
                                               createB(),
                                               createU());
             
        for (int n = 1; n < inputs.size(); ++n)
        {
            Matrix corrected = filter.timeUpdate().messurementUpdate(createX(inputs, n));
            output.add(decodeX(corrected, inputs.get(n).t));
        }

        return output;
    }
    
    /**
     * TODO: correctly decode the state matrix into a point.
     * (that is, retrieve a and v)
     * 
     * @param X the corrected state matrix
     * @return the corrected value of (x,y,t)
     */
    private static Point decodeX(Matrix X, double t)
    {
        //return new Point(X.get(0, 0), X.get(0, 1), t, 0, 0);
        throw new UnsupportedOperationException();
    }
    
    /**
     * TODO: correctly encode the point (velocity, acceleration, etc.) into a matrix.
     * 
     * Vector x = {x, y, vx , vy , ax, ay}
     * 
     * @param points 
     * @param currentIndex index of the point the filter is currently looking at
     * @return the input vector X
     */
    private static Matrix createX(List<Point> points, int currentIndex)
    {
        Matrix x = new Matrix(1, 6);
        
        List<Point> updatedPoints = computeVAndA(points);
        
        x.set(0, 0, updatedPoints.get(currentIndex).x);
        x.set(0, 1, updatedPoints.get(currentIndex).y);
        x.set(0, 2, updatedPoints.get(currentIndex).vx);
        x.set(0, 3, updatedPoints.get(currentIndex).vy);
        x.set(0, 4, updatedPoints.get(currentIndex).ax);
        x.set(0, 5, updatedPoints.get(currentIndex).ay);
        
        return x;
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
                                 points.get(i).t,
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
    private static Matrix createA(List<Point> inputs, int currentIndex)
    {
        throw new UnsupportedOperationException();
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
        double m6x6[][] = new double[VARS_COUNT][VARS_COUNT];
        
        for (int n = 0; n < VARS_COUNT; ++n)
            Arrays.fill(m6x6[n], 0);

        return Matrix.constructWithCopy(m6x6);
    }

    /**
     * TODO:
     * correctly determine this
     * 
     * @return the process noise covariance matrix
     */
    private static Matrix getQ()
    {
        throw new UnsupportedOperationException();
    }

    /**
     * 
     * @return the [1 by n] observation matrix: [1 0 0 0 .... 0 ] 
     */
    private static Matrix getH()
    {
        Matrix H = new Matrix(1,6);
        H.set(0, 0, 1);
        return H;
    }
    
    /**
     * TODO: determine the value of B
     * (could take an argument)
     * @return 
     */
    private static Matrix createB()
    {
        throw new UnsupportedOperationException();
    }
    
    /**
     * TODO: similar to B
     * 
     * @return 
     */
    private static Matrix createU()
    {
        throw new UnsupportedOperationException();
    }
    
    private static final int VARS_COUNT = 6;
}
