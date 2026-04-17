package javax.vecmath;

public final class VecmathUtils {

    public static void assignFrom(Quat4d target, Matrix3d source) {
        // Source matrix is assumed to be orthonormalized.
        var tr = source.m00 + source.m11 + source.m22;

        if (tr > 0) {
            var r = Math.sqrt(1.0 + tr);
            var s = r * 2.0;
            target.x = (source.m21 - source.m12) / s;
            target.y = (source.m02 - source.m20) / s;
            target.z = (source.m10 - source.m01) / s;
            target.w = r * 0.5;
        } else if (source.m00 > source.m11 && source.m00 > source.m22) {
            var r = Math.sqrt(1.0 + source.m00 - source.m11 - source.m22);
            var s = r * 2.0;
            target.x = r * 0.5;
            target.y = (source.m01 + source.m10) / s;
            target.z = (source.m02 + source.m20) / s;
            target.w = (source.m21 - source.m12) / s;
        } else if (source.m11 > source.m22) {
            var r = Math.sqrt(1.0 + source.m11 - source.m00 - source.m22);
            var s = r * 2.0;
            target.x = (source.m01 + source.m10) / s;
            target.y = r * 0.5;
            target.z = (source.m12 + source.m21) / s;
            target.w = (source.m02 - source.m20) / s;
        } else {
            var r = Math.sqrt(1.0 + source.m22 - source.m00 - source.m11);
            var s = r * 2.0;
            target.x = (source.m02 + source.m20) / s;
            target.y = (source.m12 + source.m21) / s;
            target.z = r * 0.5;
            target.w = (source.m10 - source.m01) / s;
        }
    }

    public static double[] getRotationScaleMatrixArray(Matrix3d matrix) {
        var matrixArray = new double[9];

        matrixArray[0] = matrix.m00;
        matrixArray[1] = matrix.m01;
        matrixArray[2] = matrix.m02;

        matrixArray[3] = matrix.m10;
        matrixArray[4] = matrix.m11;
        matrixArray[5] = matrix.m12;

        matrixArray[6] = matrix.m20;
        matrixArray[7] = matrix.m21;
        matrixArray[8] = matrix.m22;

        return matrixArray;
    }

    public static double[] getRotationScaleMatrixArray(Matrix4d matrix) {
        var matrixArray = new double[9];

        matrixArray[0] = matrix.m00;
        matrixArray[1] = matrix.m01;
        matrixArray[2] = matrix.m02;

        matrixArray[3] = matrix.m10;
        matrixArray[4] = matrix.m11;
        matrixArray[5] = matrix.m12;

        matrixArray[6] = matrix.m20;
        matrixArray[7] = matrix.m21;
        matrixArray[8] = matrix.m22;

        return matrixArray;
    }

    public static void extractRotationScale(double[] rotationScaleMatrixArray, Matrix3d outRotationMatrix, Tuple3d outScale) {
        assert(rotationScaleMatrixArray.length == 9);
        var scaleArray = new double[3];
        var rotationMatrixArray = new double[9];
        Matrix3d.compute_svd(rotationScaleMatrixArray, scaleArray, rotationMatrixArray);
        outRotationMatrix.set(rotationMatrixArray);
        outScale.set(scaleArray);
    }

    public static void extractRotationScale(double[] rotationScaleMatrixArray, Quat4d outRotation, Tuple3d outScale) {
        var rotationMatrix = new Matrix3d();
        extractRotationScale(rotationScaleMatrixArray, rotationMatrix, outScale);
        assignFrom(outRotation, rotationMatrix);
    }

    public static void extractRotationScale(Matrix3d matrix, Matrix3d outRotationMatrix, Tuple3d outScale) {
        var rotationScaleMatrixArray = getRotationScaleMatrixArray(matrix);
        extractRotationScale(rotationScaleMatrixArray, outRotationMatrix, outScale);
    }

    public static void extractRotationScale(Matrix3d matrix, Quat4d outRotation, Tuple3d outScale) {
        var rotationScaleMatrixArray = getRotationScaleMatrixArray(matrix);
        extractRotationScale(rotationScaleMatrixArray, outRotation, outScale);
    }

    public static void extractRotationScale(Matrix4d matrix, Matrix3d outRotationMatrix, Tuple3d outScale) {
        var rotationScaleMatrixArray = getRotationScaleMatrixArray(matrix);
        extractRotationScale(rotationScaleMatrixArray, outRotationMatrix, outScale);
    }

    public static void extractRotationScale(Matrix4d matrix, Quat4d outRotation, Tuple3d outScale) {
        var rotationScaleMatrixArray = getRotationScaleMatrixArray(matrix);
        extractRotationScale(rotationScaleMatrixArray, outRotation, outScale);
    }

    public static void extractTranslation(Matrix4d matrix, Tuple3d outTranslation) {
        var fourthColumn = new Vector4d();
        matrix.getColumn(3, fourthColumn);
        outTranslation.set(fourthColumn.x, fourthColumn.y, fourthColumn.z);
    }

    public static void extractTranslationRotationScale(Matrix4d matrix, Tuple3d outTranslation, Matrix3d outRotationMatrix, Tuple3d outScale) {
        extractTranslation(matrix, outTranslation);
        extractRotationScale(matrix, outRotationMatrix, outScale);
    }

    public static void extractTranslationRotationScale(Matrix4d matrix, Tuple3d outTranslation, Quat4d outRotation, Tuple3d outScale) {
        extractTranslation(matrix, outTranslation);
        extractRotationScale(matrix, outRotation, outScale);
    }
}
