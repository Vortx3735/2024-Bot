package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class RootFinder {

    public static List<Double> rootFinder(List<Double> equation, double error) {
        List<List<Double>> derivatives = new ArrayList<>();
        derivatives.add(equation);
        // Take derivatives of the equation down to the quadratic
        for (int i = 0; i < equation.size() - 3; i++) {
            derivatives.add(takeDerivative(derivatives.get(i)));
        }
        // Find solutions to the quadratic
        List<Double> roots = quadraticFormula(derivatives.get(derivatives.size() - 1));
        Collections.sort(roots);
        if (roots.size() != 2) {
            System.out.println("Could not find roots :(");
            return null;
        }
        derivatives.remove(derivatives.size() - 1); // Remove the quadratic now that we have its roots
        for (int i = derivatives.size() - 1; i >= 0; i--) {
            List<Double> newRoots = new ArrayList<>();
            newRoots.add(newtonsMethod(derivatives.get(i), roots.get(0) - 20, error));
            for (int j = 0; j < roots.size() - 1; j++) {
                Double root;
                if (functionOutput(derivatives.get(i), roots.get(j)) > functionOutput(derivatives.get(i), roots.get(j + 1))) {
                    root = binarySearchDescending(derivatives.get(i), roots.get(j), roots.get(j + 1), error);
                } else {
                    root = binarySearchAscending(derivatives.get(i), roots.get(j), roots.get(j + 1), error);
                }
                if (root != null) {
                    newRoots.add(root);
                }
            }
            newRoots.add(newtonsMethod(derivatives.get(i), roots.get(roots.size() - 1) + 20, error));
            roots = newRoots;
        }
        return roots;
    }

    private static List<Double> takeDerivative(List<Double> equation) {
        List<Double> derivative = new ArrayList<>();
        int largestPower = equation.size() - 1;
        for (int i = 0; i < largestPower; i++) {
            int exponent = largestPower - i;
            derivative.add(exponent * equation.get(i));
        }
        return derivative;
    }

    private static List<Double> quadraticFormula(List<Double> equation) {
        double a = equation.get(0);
        double b = equation.get(1);
        double c = equation.get(2);
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return Collections.emptyList();
        } else if (discriminant == 0) {
            return Collections.singletonList(-b / (2 * a));
        } else {
            double squareRoot = Math.sqrt(discriminant);
            return Arrays.asList((-b + squareRoot) / (2 * a), (-b - squareRoot) / (2 * a));
        }
    }

    private static Double binarySearchDescending(List<Double> equation, double left, double right, double error) {
        while (left <= right) {
            double mid = (left + right) / 2;
            double output = functionOutput(equation, mid);
            if (withinRange(output, 0, error)) {
                return mid;
            } else if (output > 0) {
                left = mid + error / 100;
            } else {
                right = mid - error / 100;
            }
        }
        return null;
    }

    private static Double binarySearchAscending(List<Double> equation, double left, double right, double error) {
        while (left <= right) {
            double mid = (left + right) / 2;
            double output = functionOutput(equation, mid);
            if (withinRange(output, 0, error)) {
                return mid;
            } else if (output < 0) {
                left = mid + error / 100;
            } else {
                right = mid - error / 100;
            }
        }
        return null;
    }

    private static double functionOutput(List<Double> equation, double input) {
        double result = 0;
        int length = equation.size();
        for (int i = 0; i < length; i++) {
            result += equation.get(length - i - 1) * Math.pow(input, i);
        }
        return result;
    }

    private static boolean withinRange(double num, double target, double range) {
        return num <= target + range && num >= target - range;
    }

    private static Double newtonsMethod(List<Double> equation, double guess, double error) {
        List<Double> derivative = takeDerivative(equation);
        double delta = Math.abs(functionOutput(equation, guess));
        while (delta > error) {
            guess = guess - functionOutput(equation, guess) / functionOutput(derivative, guess);
            delta = Math.abs(functionOutput(equation, guess));
        }
        return guess;
    }
}