package org.firstinspires.ftc.suitbots;

public class SVC {
    private enum Kernel {LINEAR, POLY, RBF, SIGMOID}

    private int nClasses;
    private int nRows;
    private int[] classes;
    private double[][] vectors;
    private double[][] coefficients;
    private double[] intercepts;
    private int[] weights;
    private Kernel kernel;
    private double gamma;
    private double coef0;
    private double degree;

    public SVC(int nClasses, int nRows, double[][] vectors, double[][] coefficients, double[] intercepts, int[] weights, String kernel, double gamma, double coef0, double degree) {
        this.nClasses = nClasses;
        this.classes = new int[nClasses];
        for (int i = 0; i < nClasses; i++) {
            this.classes[i] = i;
        }
        this.nRows = nRows;
        this.vectors = vectors;
        this.coefficients = coefficients;
        this.intercepts = intercepts;
        this.weights = weights;
        this.kernel = Kernel.valueOf(kernel.toUpperCase());
        this.gamma = gamma;
        this.coef0 = coef0;
        this.degree = degree;
    }

    public int predict(double[] features) {
        double[] kernels = new double[vectors.length];
        double kernel;
        switch (this.kernel) {
            case LINEAR:
                // <x,x'>
                for (int i = 0; i < this.vectors.length; i++) {
                    kernel = 0.;
                    for (int j = 0; j < this.vectors[i].length; j++) {
                        kernel += this.vectors[i][j] * features[j];
                    }
                    kernels[i] = kernel;
                }
                break;
            case POLY:
                // (y<x,x'>+r)^d
                for (int i = 0; i < this.vectors.length; i++) {
                    kernel = 0.;
                    for (int j = 0; j < this.vectors[i].length; j++) {
                        kernel += this.vectors[i][j] * features[j];
                    }
                    kernels[i] = Math.pow((this.gamma * kernel) + this.coef0, this.degree);
                }
                break;
            case RBF:
                // exp(-y|x-x'|^2)
                for (int i = 0; i < this.vectors.length; i++) {
                    kernel = 0.;
                    for (int j = 0; j < this.vectors[i].length; j++) {
                        kernel += Math.pow(this.vectors[i][j] - features[j], 2);
                    }
                    kernels[i] = Math.exp(-this.gamma * kernel);
                }
                break;
            case SIGMOID:
                // tanh(y<x,x'>+r)
                for (int i = 0; i < this.vectors.length; i++) {
                    kernel = 0.;
                    for (int j = 0; j < this.vectors[i].length; j++) {
                        kernel += this.vectors[i][j] * features[j];
                    }
                    kernels[i] = Math.tanh((this.gamma * kernel) + this.coef0);
                }
                break;
        }
        int[] starts = new int[this.nRows];
        for (int i = 0; i < this.nRows; i++) {
            if (i != 0) {
                int start = 0;
                for (int j = 0; j < i; j++) {
                    start += this.weights[j];
                }
                starts[i] = start;
            } else {
                starts[0] = 0;
            }
        }
        int[] ends = new int[this.nRows];
        for (int i = 0; i < this.nRows; i++) {
            ends[i] = this.weights[i] + starts[i];
        }
        if (this.nClasses == 2) {
            for (int i = 0; i < kernels.length; i++) {
                kernels[i] = -kernels[i];
            }
            double decision = 0.;
            for (int k = starts[1]; k < ends[1]; k++) {
                decision += kernels[k] * this.coefficients[0][k];
            }
            for (int k = starts[0]; k < ends[0]; k++) {
                decision += kernels[k] * this.coefficients[0][k];
            }
            decision += this.intercepts[0];
            if (decision > 0) {
                return 0;
            }
            return 1;
        }
        double[] decisions = new double[this.intercepts.length];
        for (int i = 0, d = 0, l = this.nRows; i < l; i++) {
            for (int j = i + 1; j < l; j++) {
                double tmp = 0.;
                for (int k = starts[j]; k < ends[j]; k++) {
                    tmp += this.coefficients[i][k] * kernels[k];
                }
                for (int k = starts[i]; k < ends[i]; k++) {
                    tmp += this.coefficients[j - 1][k] * kernels[k];
                }
                decisions[d] = tmp + this.intercepts[d];
                d++;
            }
        }
        int[] votes = new int[this.intercepts.length];
        for (int i = 0, d = 0, l = this.nRows; i < l; i++) {
            for (int j = i + 1; j < l; j++) {
                votes[d] = decisions[d] > 0 ? i : j;
                d++;
            }
        }
        int[] amounts = new int[this.nClasses];
        for (int i = 0, l = votes.length; i < l; i++) {
            amounts[votes[i]] += 1;
        }
        int classVal = -1, classIdx = -1;
        for (int i = 0, l = amounts.length; i < l; i++) {
            if (amounts[i] > classVal) {
                classVal = amounts[i];
                classIdx = i;
            }
        }
        return this.classes[classIdx];
    }

    public static SVC svc() {
        double[][] vectors = {{308.26, 148.64, 116.56, 666.61, -1.0, -1.0, -1.0, -1.0}, {307.64, 147.87, 121.16, 637.88, -1.0, -1.0, -1.0, -1.0}, {307.82, 145.47, 131.47, 656.52, -1.0, -1.0, -1.0, -1.0}, {306.09, 162.0, 6.72, 323.82, -1.0, -1.0, -1.0, -1.0}, {306.02, 162.3, 7.08, 323.37, -1.0, -1.0, -1.0, -1.0}, {306.01, 162.26, 7.13, 323.49, -1.0, -1.0, -1.0, -1.0}, {308.72, 151.52, -2.21, 514.95, -1.0, -1.0, -1.0, -1.0}, {308.89, 149.03, -11.59, 514.35, -1.0, -1.0, -1.0, -1.0}, {308.83, 149.07, -12.12, 495.81, -1.0, -1.0, -1.0, -1.0}, {308.86, 149.09, -12.14, 495.82, -1.0, -1.0, -1.0, -1.0}, {306.41, 159.39, -9.95, 324.85, 309.77, 158.15, 323.27, 604.52}, {306.46, 159.27, -9.86, 325.33, 310.47, 158.56, 330.62, 612.41}, {306.58, 159.31, -10.01, 325.37, 310.64, 158.43, 325.65, 603.22}};
        double[][] coefficients = {{0.7520953613686517, 0.78101156470057, 0.1944538970000209, -0.38549706475491274, -0.0, -0.38889058680060357, -0.4624802129254125, -0.018445973168804468, -0.47224698541950927, -0.0, -0.5581997261919289, -0.6013644587727279, -0.049090951534339844}, {0.5267726218549854, 0.5463557142875677, 0.13552680035644352, 0.11098167944166847, 0.5326195199994582, 0.0, 0.3832739603529584, 0.016760178436996443, 0.01582691166516831, 0.3763387045306869, -0.6579545957595723, -0.7155689443624219, -0.062277414304942456}};
        double[] intercepts = {-0.22588935285603692, 0.14201693433186635, 0.3564438429042772};
        int[] weights = {3, 7, 3};        // Prediction:
        SVC clf = new SVC(3, 3, vectors, coefficients, intercepts, weights, "rbf", 0.001, 0.0, 3);
        return clf;
    }

    public static void main(String[] args) {
        if (args.length == 8) {            // Features:
            double[] features = new double[args.length];
            for (int i = 0, l = args.length; i < l; i++) {
                features[i] = Double.parseDouble(args[i]);
            }            // Parameters:
            double[][] vectors = {{308.26, 148.64, 116.56, 666.61, -1.0, -1.0, -1.0, -1.0}, {307.64, 147.87, 121.16, 637.88, -1.0, -1.0, -1.0, -1.0}, {307.82, 145.47, 131.47, 656.52, -1.0, -1.0, -1.0, -1.0}, {306.09, 162.0, 6.72, 323.82, -1.0, -1.0, -1.0, -1.0}, {306.02, 162.3, 7.08, 323.37, -1.0, -1.0, -1.0, -1.0}, {306.01, 162.26, 7.13, 323.49, -1.0, -1.0, -1.0, -1.0}, {308.72, 151.52, -2.21, 514.95, -1.0, -1.0, -1.0, -1.0}, {308.89, 149.03, -11.59, 514.35, -1.0, -1.0, -1.0, -1.0}, {308.83, 149.07, -12.12, 495.81, -1.0, -1.0, -1.0, -1.0}, {308.86, 149.09, -12.14, 495.82, -1.0, -1.0, -1.0, -1.0}, {306.41, 159.39, -9.95, 324.85, 309.77, 158.15, 323.27, 604.52}, {306.46, 159.27, -9.86, 325.33, 310.47, 158.56, 330.62, 612.41}, {306.58, 159.31, -10.01, 325.37, 310.64, 158.43, 325.65, 603.22}};
            double[][] coefficients = {{0.7520953613686517, 0.78101156470057, 0.1944538970000209, -0.38549706475491274, -0.0, -0.38889058680060357, -0.4624802129254125, -0.018445973168804468, -0.47224698541950927, -0.0, -0.5581997261919289, -0.6013644587727279, -0.049090951534339844}, {0.5267726218549854, 0.5463557142875677, 0.13552680035644352, 0.11098167944166847, 0.5326195199994582, 0.0, 0.3832739603529584, 0.016760178436996443, 0.01582691166516831, 0.3763387045306869, -0.6579545957595723, -0.7155689443624219, -0.062277414304942456}};
            double[] intercepts = {-0.22588935285603692, 0.14201693433186635, 0.3564438429042772};
            int[] weights = {3, 7, 3};            // Prediction:
            SVC clf = new SVC(3, 3, vectors, coefficients, intercepts, weights, "rbf", 0.001, 0.0, 3);
            int estimation = clf.predict(features);
            System.out.println(estimation);
        }
    }
}