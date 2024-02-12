// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4400.Util.Interpolation;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingDouble implements Interpolable<InterpolatingDouble>, InverseInterpolable<InterpolatingDouble>,
        Comparable<InterpolatingDouble> {
    public Double value;

    public InterpolatingDouble(Double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
        double dydx = other.value - value;
        Double searchY = dydx * x + value;
        return new InterpolatingDouble(searchY);
    }

    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        double upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }
        double query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }
        return query_to_lower / upper_to_lower;
    }

    @Override
    public int compareTo(InterpolatingDouble other) {
        return value.compareTo(other.value);
    }
}