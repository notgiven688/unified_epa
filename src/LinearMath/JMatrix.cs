/* Copyright <2021> <Thorben Linneweber>
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
* 
*/
using System;
using System.Runtime.CompilerServices;

namespace GJKEPADemo
{
    /// <summary>
    /// 3x3 Matrix. Member of the math namespace, so every method
    /// has it's 'by reference' equivalent to speed up time critical
    /// math operations.
    /// </summary>
    public struct JMatrix
    {
        /// <summary>
        /// M11
        /// </summary>
        public double M11; // 1st row vector
        /// <summary>
        /// M12
        /// </summary>
        public double M12;
        /// <summary>
        /// M13
        /// </summary>
        public double M13;
        /// <summary>
        /// M21
        /// </summary>
        public double M21; // 2nd row vector
        /// <summary>
        /// M22
        /// </summary>
        public double M22;
        /// <summary>
        /// M23
        /// </summary>
        public double M23;
        /// <summary>
        /// M31
        /// </summary>
        public double M31; // 3rd row vector
        /// <summary>
        /// M32
        /// </summary>
        public double M32;
        /// <summary>
        /// M33
        /// </summary>
        public double M33;

        internal static JMatrix InternalIdentity;

        /// <summary>
        /// Identity matrix.
        /// </summary>
        public static readonly JMatrix Identity;
        public static readonly JMatrix Zero;

        static JMatrix()
        {
            Zero = new JMatrix();

            Identity = new JMatrix();
            Identity.M11 = 1.0f;
            Identity.M22 = 1.0f;
            Identity.M33 = 1.0f;

            InternalIdentity = Identity;
        }

        public override string ToString()
        {
            return M11.ToString() + " " + M12.ToString() + " "  + M13.ToString() + Environment.NewLine +
            M21.ToString() + " " + M22.ToString() + " "  + M23.ToString() + Environment.NewLine +
            M31.ToString() + " " + M32.ToString() + " "  + M33.ToString();
        }

        public static JMatrix CreateRotationX(double radians)
        {
            JMatrix matrix;
            double num2 = (double)Math.Cos((double)radians);
            double num = (double)Math.Sin((double)radians);
            matrix.M11 = 1f;
            matrix.M12 = 0f;
            matrix.M13 = 0f;
            matrix.M21 = 0f;
            matrix.M22 = num2;
            matrix.M23 = num;
            matrix.M31 = 0f;
            matrix.M32 = -num;
            matrix.M33 = num2;
            return matrix;
        }

        public static void CreateRotationX(double radians, out JMatrix result)
        {
            double num2 = (double)Math.Cos((double)radians);
            double num = (double)Math.Sin((double)radians);
            result.M11 = 1f;
            result.M12 = 0f;
            result.M13 = 0f;
            result.M21 = 0f;
            result.M22 = num2;
            result.M23 = num;
            result.M31 = 0f;
            result.M32 = -num;
            result.M33 = num2;
        }

        public static JMatrix CreateRotationY(double radians)
        {
            JMatrix matrix;
            double num2 = (double)Math.Cos((double)radians);
            double num = (double)Math.Sin((double)radians);
            matrix.M11 = num2;
            matrix.M12 = 0f;
            matrix.M13 = -num;
            matrix.M21 = 0f;
            matrix.M22 = 1f;
            matrix.M23 = 0f;
            matrix.M31 = num;
            matrix.M32 = 0f;
            matrix.M33 = num2;
            return matrix;
        }

        public static void CreateRotationY(double radians, out JMatrix result)
        {
            double num2 = (double)Math.Cos((double)radians);
            double num = (double)Math.Sin((double)radians);
            result.M11 = num2;
            result.M12 = 0f;
            result.M13 = -num;
            result.M21 = 0f;
            result.M22 = 1f;
            result.M23 = 0f;
            result.M31 = num;
            result.M32 = 0f;
            result.M33 = num2;
        }

        public static JMatrix CreateRotationZ(double radians)
        {
            JMatrix matrix;
            double num2 = (double)Math.Cos((double)radians);
            double num = (double)Math.Sin((double)radians);
            matrix.M11 = num2;
            matrix.M12 = num;
            matrix.M13 = 0f;
            matrix.M21 = -num;
            matrix.M22 = num2;
            matrix.M23 = 0f;
            matrix.M31 = 0f;
            matrix.M32 = 0f;
            matrix.M33 = 1f;
            return matrix;
        }


        public static void CreateRotationZ(double radians, out JMatrix result)
        {
            double num2 = (double)Math.Cos((double)radians);
            double num = (double)Math.Sin((double)radians);
            result.M11 = num2;
            result.M12 = num;
            result.M13 = 0f;
            result.M21 = -num;
            result.M22 = num2;
            result.M23 = 0f;
            result.M31 = 0f;
            result.M32 = 0f;
            result.M33 = 1f;
        }


        /// <summary>
        /// Initializes a new instance of the matrix structure.
        /// </summary>
        /// <param name="m11">m11</param>
        /// <param name="m12">m12</param>
        /// <param name="m13">m13</param>
        /// <param name="m21">m21</param>
        /// <param name="m22">m22</param>
        /// <param name="m23">m23</param>
        /// <param name="m31">m31</param>
        /// <param name="m32">m32</param>
        /// <param name="m33">m33</param>
        #region public JMatrix(double m11, double m12, double m13, double m21, double m22, double m23,double m31, double m32, double m33)
        public JMatrix(double m11, double m12, double m13, double m21, double m22, double m23,double m31, double m32, double m33)
        {
            this.M11 = m11;
            this.M12 = m12;
            this.M13 = m13;
            this.M21 = m21;
            this.M22 = m22;
            this.M23 = m23;
            this.M31 = m31;
            this.M32 = m32;
            this.M33 = m33;
        }

        public JMatrix(ref JVector row1, ref JVector row2, ref JVector row3)
        {
            this.M11 = row1.X;
            this.M12 = row1.Y;
            this.M13 = row1.Z;
            this.M21 = row2.X;
            this.M22 = row2.Y;
            this.M23 = row2.Z;
            this.M31 = row3.X;
            this.M32 = row3.Y;
            this.M33 = row3.Z;
        }
        #endregion

        /// <summary>
        /// Gets the determinant of the matrix.
        /// </summary>
        /// <returns>The determinant of the matrix.</returns>
        #region public double Determinant()
        //public double Determinant()
        //{
        //    return M11 * M22 * M33 -M11 * M23 * M32 -M12 * M21 * M33 +M12 * M23 * M31 + M13 * M21 * M32 - M13 * M22 * M31;
        //}
        #endregion

        /// <summary>
        /// Multiply two matrices. Notice: matrix multiplication is not commutative.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <returns>The product of both matrices.</returns>
        #region public static JMatrix Multiply(JMatrix matrix1, JMatrix matrix2)
        public static JMatrix Multiply(JMatrix matrix1, JMatrix matrix2)
        {
            JMatrix result;
            JMatrix.Multiply(ref matrix1, ref matrix2, out result);
            return result;
        }

        /// <summary>
        /// Multiply two matrices. Notice: matrix multiplication is not commutative.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <param name="result">The product of both matrices.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref JMatrix matrix1, ref JMatrix matrix2, out JMatrix result)
        {
            double num0 = ((matrix1.M11 * matrix2.M11) + (matrix1.M12 * matrix2.M21)) + (matrix1.M13 * matrix2.M31);
            double num1 = ((matrix1.M11 * matrix2.M12) + (matrix1.M12 * matrix2.M22)) + (matrix1.M13 * matrix2.M32);
            double num2 = ((matrix1.M11 * matrix2.M13) + (matrix1.M12 * matrix2.M23)) + (matrix1.M13 * matrix2.M33);
            double num3 = ((matrix1.M21 * matrix2.M11) + (matrix1.M22 * matrix2.M21)) + (matrix1.M23 * matrix2.M31);
            double num4 = ((matrix1.M21 * matrix2.M12) + (matrix1.M22 * matrix2.M22)) + (matrix1.M23 * matrix2.M32);
            double num5 = ((matrix1.M21 * matrix2.M13) + (matrix1.M22 * matrix2.M23)) + (matrix1.M23 * matrix2.M33);
            double num6 = ((matrix1.M31 * matrix2.M11) + (matrix1.M32 * matrix2.M21)) + (matrix1.M33 * matrix2.M31);
            double num7 = ((matrix1.M31 * matrix2.M12) + (matrix1.M32 * matrix2.M22)) + (matrix1.M33 * matrix2.M32);
            double num8 = ((matrix1.M31 * matrix2.M13) + (matrix1.M32 * matrix2.M23)) + (matrix1.M33 * matrix2.M33);

            result.M11 = num0;
            result.M12 = num1;
            result.M13 = num2;
            result.M21 = num3;
            result.M22 = num4;
            result.M23 = num5;
            result.M31 = num6;
            result.M32 = num7;
            result.M33 = num8;
        }
        #endregion

        /// <summary>
        /// Matrices are added.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <returns>The sum of both matrices.</returns>
        #region public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
        public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
        {
            JMatrix result;
            JMatrix.Add(ref matrix1, ref matrix2, out result);
            return result;
        }

        /// <summary>
        /// Matrices are added.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <param name="result">The sum of both matrices.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref JMatrix matrix1, ref JMatrix matrix2, out JMatrix result)
        {
            result.M11 = matrix1.M11 + matrix2.M11;
            result.M12 = matrix1.M12 + matrix2.M12;
            result.M13 = matrix1.M13 + matrix2.M13;
            result.M21 = matrix1.M21 + matrix2.M21;
            result.M22 = matrix1.M22 + matrix2.M22;
            result.M23 = matrix1.M23 + matrix2.M23;
            result.M31 = matrix1.M31 + matrix2.M31;
            result.M32 = matrix1.M32 + matrix2.M32;
            result.M33 = matrix1.M33 + matrix2.M33;
        }
        #endregion

        /// <summary>
        /// Calculates the inverse of a give matrix.
        /// </summary>
        /// <param name="matrix">The matrix to invert.</param>
        /// <returns>The inverted JMatrix.</returns>
        #region public static JMatrix Inverse(JMatrix matrix)
        public static JMatrix Invert(JMatrix matrix)
        {
            JMatrix result;
            JMatrix.Invert(ref matrix, out result);
            return result;
        }

        public double Determinant()
        {
            return M11 * M22 * M33 + M12 * M23 * M31 + M13 * M21 * M32 -
                   M31 * M22 * M13 - M32 * M23 * M11 - M33 * M21 * M12;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(ref JMatrix matrix, out JMatrix result)
        {
            double determinantInverse = 1 / matrix.Determinant();
            double m11 = (matrix.M22 * matrix.M33 - matrix.M23 * matrix.M32) * determinantInverse;
            double m12 = (matrix.M13 * matrix.M32 - matrix.M33 * matrix.M12) * determinantInverse;
            double m13 = (matrix.M12 * matrix.M23 - matrix.M22 * matrix.M13) * determinantInverse;

            double m21 = (matrix.M23 * matrix.M31 - matrix.M21 * matrix.M33) * determinantInverse;
            double m22 = (matrix.M11 * matrix.M33 - matrix.M13 * matrix.M31) * determinantInverse;
            double m23 = (matrix.M13 * matrix.M21 - matrix.M11 * matrix.M23) * determinantInverse;

            double m31 = (matrix.M21 * matrix.M32 - matrix.M22 * matrix.M31) * determinantInverse;
            double m32 = (matrix.M12 * matrix.M31 - matrix.M11 * matrix.M32) * determinantInverse;
            double m33 = (matrix.M11 * matrix.M22 - matrix.M12 * matrix.M21) * determinantInverse;

            result.M11 = m11;
            result.M12 = m12;
            result.M13 = m13;

            result.M21 = m21;
            result.M22 = m22;
            result.M23 = m23;

            result.M31 = m31;
            result.M32 = m32;
            result.M33 = m33;
        }

        #endregion

        /// <summary>
        /// Multiply a matrix by a scalefactor.
        /// </summary>
        /// <param name="matrix1">The matrix.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <returns>A JMatrix multiplied by the scale factor.</returns>
        #region public static JMatrix Multiply(JMatrix matrix1, double scaleFactor)
        public static JMatrix Multiply(JMatrix matrix1, double scaleFactor)
        {
            JMatrix result;
            JMatrix.Multiply(ref matrix1, scaleFactor, out result);
            return result;
        }

        /// <summary>
        /// Multiply a matrix by a scalefactor.
        /// </summary>
        /// <param name="matrix1">The matrix.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <param name="result">A JMatrix multiplied by the scale factor.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref JMatrix matrix1, double scaleFactor, out JMatrix result)
        {
            double num = scaleFactor;
            result.M11 = matrix1.M11 * num;
            result.M12 = matrix1.M12 * num;
            result.M13 = matrix1.M13 * num;
            result.M21 = matrix1.M21 * num;
            result.M22 = matrix1.M22 * num;
            result.M23 = matrix1.M23 * num;
            result.M31 = matrix1.M31 * num;
            result.M32 = matrix1.M32 * num;
            result.M33 = matrix1.M33 * num;
        }
        #endregion

        /// <summary>
        /// Creates the transposed matrix.
        /// </summary>
        /// <param name="matrix">The matrix which should be transposed.</param>
        /// <returns>The transposed JMatrix.</returns>
        #region public static JMatrix Transpose(JMatrix matrix)
        public static JMatrix Transpose(JMatrix matrix)
        {
            JMatrix result;
            JMatrix.Transpose(ref matrix, out result);
            return result;
        }

        /// <summary>
        /// Creates the transposed matrix.
        /// </summary>
        /// <param name="matrix">The matrix which should be transposed.</param>
        /// <param name="result">The transposed JMatrix.</param>
        public static void Transpose(ref JMatrix matrix, out JMatrix result)
        {
            result.M11 = matrix.M11;
            result.M12 = matrix.M21;
            result.M13 = matrix.M31;
            result.M21 = matrix.M12;
            result.M22 = matrix.M22;
            result.M23 = matrix.M32;
            result.M31 = matrix.M13;
            result.M32 = matrix.M23;
            result.M33 = matrix.M33;
        }
        #endregion

        /// <summary>
        /// Multiplies two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The product of both values.</returns>
        #region public static JMatrix operator *(JMatrix value1,JMatrix value2)
        public static JMatrix operator *(JMatrix value1,JMatrix value2)
        {
            JMatrix result; JMatrix.Multiply(ref value1, ref value2, out result);
            return result;
        }
        #endregion


        public double Trace()
        {
            return this.M11 + this.M22 + this.M33;
        }

        /// <summary>
        /// Adds two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The sum of both values.</returns>
        #region public static JMatrix operator +(JMatrix value1, JMatrix value2)
        public static JMatrix operator +(JMatrix value1, JMatrix value2)
        {
            JMatrix result; JMatrix.Add(ref value1, ref value2, out result);
            return result;
        }
        #endregion

        /// <summary>
        /// Subtracts two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The difference of both values.</returns>
        #region public static JMatrix operator -(JMatrix value1, JMatrix value2)
        public static JMatrix operator -(JMatrix value1, JMatrix value2)
        {
            JMatrix result; JMatrix.Multiply(ref value2, -1.0f, out value2);
            JMatrix.Add(ref value1, ref value2, out result);
            return result;
        }
        #endregion


        /// <summary>
        /// Creates a matrix which rotates around the given axis by the given angle.
        /// </summary>
        /// <param name="axis">The axis.</param>
        /// <param name="angle">The angle.</param>
        /// <param name="result">The resulting rotation matrix</param>
        #region public static void CreateFromAxisAngle(ref JVector axis, double angle, out JMatrix result)
        public static void CreateFromAxisAngle(ref JVector axis, double angle, out JMatrix result)
        {
            double x = axis.X;
            double y = axis.Y;
            double z = axis.Z;
            double num2 = (double)Math.Sin((double)angle);
            double num = (double)Math.Cos((double)angle);
            double num11 = x * x;
            double num10 = y * y;
            double num9 = z * z;
            double num8 = x * y;
            double num7 = x * z;
            double num6 = y * z;
            result.M11 = num11 + (num * (1f - num11));
            result.M12 = (num8 - (num * num8)) + (num2 * z);
            result.M13 = (num7 - (num * num7)) - (num2 * y);
            result.M21 = (num8 - (num * num8)) - (num2 * z);
            result.M22 = num10 + (num * (1f - num10));
            result.M23 = (num6 - (num * num6)) + (num2 * x);
            result.M31 = (num7 - (num * num7)) + (num2 * y);
            result.M32 = (num6 - (num * num6)) - (num2 * x);
            result.M33 = num9 + (num * (1f - num9));
        }

        /// <summary>
        /// Creates a matrix which rotates around the given axis by the given angle.
        /// </summary>
        /// <param name="axis">The axis.</param>
        /// <param name="angle">The angle.</param>
        /// <returns>The resulting rotation matrix</returns>
        public static JMatrix CreateFromAxisAngle(JVector axis, double angle)
        {
            JMatrix result; CreateFromAxisAngle(ref axis, angle, out result);
            return result;
        }
        #endregion
    }
}
