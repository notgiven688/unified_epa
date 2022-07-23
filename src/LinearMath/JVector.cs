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
    /// A vector structure. Member of the math 
    /// namespace, so every method has it's 'by reference' equivalent
    /// to speed up time critical math operations.
    /// </summary>
    public struct JVector
    {
        internal static JVector InternalZero;
        internal static JVector Arbitrary;

        /// <summary>The X component of the vector.</summary>
        public double X;
        /// <summary>The Y component of the vector.</summary>
        public double Y;
        /// <summary>The Z component of the vector.</summary>
        public double Z;

        #region Static readonly variables
        /// <summary>
        /// A vector with components (0,0,0);
        /// </summary>
        public static readonly JVector Zero;
        /// <summary>
        /// A vector with components (1,0,0);
        /// </summary>
        public static readonly JVector Left;
        /// <summary>
        /// A vector with components (-1,0,0);
        /// </summary>
        public static readonly JVector Right;
        /// <summary>
        /// A vector with components (0,1,0);
        /// </summary>
        public static readonly JVector Up;
        /// <summary>
        /// A vector with components (0,-1,0);
        /// </summary>
        public static readonly JVector Down;
        /// <summary>
        /// A vector with components (0,0,1);
        /// </summary>
        public static readonly JVector Backward;
        /// <summary>
        /// A vector with components (0,0,-1);
        /// </summary>
        public static readonly JVector Forward;
        /// <summary>
        /// A vector with components (1,1,1);
        /// </summary>
        public static readonly JVector One;
        /// <summary>
        /// A vector with components 
        /// (double.MinValue,double.MinValue,double.MinValue);
        /// </summary>
        public static readonly JVector MinValue;
        /// <summary>
        /// A vector with components 
        /// (double.MaxValue,double.MaxValue,double.MaxValue);
        /// </summary>
        public static readonly JVector MaxValue;
        #endregion

        #region Private static constructor
        static JVector()
        {
            One = new JVector(1, 1, 1);
            Zero = new JVector(0, 0, 0);
            Left = new JVector(1, 0, 0);
            Right = new JVector(-1, 0, 0);
            Up = new JVector(0, 1, 0);
            Down = new JVector(0, -1, 0);
            Backward = new JVector(0, 0, 1);
            Forward = new JVector(0, 0, -1);
            MinValue = new JVector(double.MinValue);
            MaxValue = new JVector(double.MaxValue);
            Arbitrary = new JVector(1, 1, 1);
            InternalZero = Zero;
        }
        #endregion

        public static JVector operator -(in JVector value)
        {
            JVector vector;
            vector.X = -value.X;
            vector.Y = -value.Y;
            vector.Z = -value.Z;
            return vector;
        }

        public double this[int i]
        {
            get
            {
                if (i == 0) return this.X;
                else if (i == 1) return this.Y;
                else return this.Z;
            }
            set
            {
                if (i == 0) this.X = value;
                else if (i == 1) this.Y = value;
                else this.Z = value;
            }
        }

        /// <summary>
        /// Constructor initializing a new instance of the structure
        /// </summary>
        /// <param name="x">The X component of the vector.</param>
        /// <param name="y">The Y component of the vector.</param>
        /// <param name="z">The Z component of the vector.</param>
        public JVector(double x, double y, double z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        /// <summary>
        /// Sets all vector component to specific values.
        /// </summary>
        /// <param name="x">The X component of the vector.</param>
        /// <param name="y">The Y component of the vector.</param>
        /// <param name="z">The Z component of the vector.</param>
        public void Set(double x, double y, double z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        /// <summary>
        /// Constructor initializing a new instance of the structure
        /// </summary>
        /// <param name="xyz">All components of the vector are set to xyz</param>
        public JVector(double xyz)
        {
            this.X = xyz;
            this.Y = xyz;
            this.Z = xyz;
        }

        /// <summary>
        /// Builds a string from the JVector.
        /// </summary>
        /// <returns>A string containing all three components.</returns>
        #region public override string ToString()
        public override string ToString()
        {
            return "X=" + X.ToString() + " Y=" + Y.ToString() + " Z=" + Z.ToString();
        }
        #endregion

        /// <summary>
        /// Tests if an object is equal to this vector.
        /// </summary>
        /// <param name="obj">The object to test.</param>
        /// <returns>Returns true if they are euqal, otherwise false.</returns>
        #region public override bool Equals(object obj)
        public override bool Equals(object obj)
        {
            if (!(obj is JVector)) return false;
            JVector other = (JVector)obj;

            return (((X == other.X) && (Y == other.Y)) && (Z == other.Z));
        }
        #endregion

        /// <summary>
        /// Tests if two JVector are equal.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>Returns true if both values are equal, otherwise false.</returns>
        #region public static bool operator ==(JVector value1, JVector value2)
        public static bool operator ==(in JVector value1, in JVector value2)
        {
            return (((value1.X == value2.X) && (value1.Y == value2.Y)) && (value1.Z == value2.Z));
        }
        #endregion

        /// <summary>
        /// Tests if two JVector are not equal.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>Returns false if both values are equal, otherwise true.</returns>
        #region public static bool operator !=(JVector value1, JVector value2)
        public static bool operator !=(in JVector value1, in JVector value2)
        {
            if ((value1.X == value2.X) && (value1.Y == value2.Y))
            {
                return (value1.Z != value2.Z);
            }
            return true;
        }
        #endregion

        /// <summary>
        /// Gets a vector with the minimum x,y and z values of both vectors.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>A vector with the minimum x,y and z values of both vectors.</returns>
        #region public static JVector Min(JVector value1, JVector value2)

        public static JVector Min(in JVector value1, in JVector value2)
        {
            JVector.Min(value1, value2, out JVector result);
            return result;
        }

        /// <summary>
        ///  ets a vector with the minimum x,y and z values of both vectors.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <param name="result">A vector with the minimum x,y and z values of both vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(in JVector value1, in JVector value2, out JVector result)
        {
            result.X = (value1.X < value2.X) ? value1.X : value2.X;
            result.Y = (value1.Y < value2.Y) ? value1.Y : value2.Y;
            result.Z = (value1.Z < value2.Z) ? value1.Z : value2.Z;
        }
        #endregion

        /// <summary>
        /// Gets a vector with the maximum x,y and z values of both vectors.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>A vector with the maximum x,y and z values of both vectors.</returns>
        #region public static JVector Max(JVector value1, JVector value2)
        public static JVector Max(in JVector value1, in JVector value2)
        {
            JVector.Max(value1, value2, out JVector result);
            return result;
        }

        /// <summary>
        /// Gets a vector with the maximum x,y and z values of both vectors.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <param name="result">A vector with the maximum x,y and z values of both vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(in JVector value1, in JVector value2, out JVector result)
        {
            result.X = (value1.X > value2.X) ? value1.X : value2.X;
            result.Y = (value1.Y > value2.Y) ? value1.Y : value2.Y;
            result.Z = (value1.Z > value2.Z) ? value1.Z : value2.Z;
        }
        #endregion

        /// <summary>
        /// Sets the length of the vector to zero.
        /// </summary>
        #region public void MakeZero()
        public void MakeZero()
        {
            X = 0.0f;
            Y = 0.0f;
            Z = 0.0f;
        }
        #endregion

        /// <summary>
        /// Checks if the length of the vector is zero.
        /// </summary>
        /// <returns>Returns true if the vector is zero, otherwise false.</returns>
        #region public bool IsZero()
        public bool IsZero()
        {
            return (this.LengthSquared() == 0.0f);
        }

        #endregion

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <returns>The transformed vector.</returns>
        #region public static JVector Transform(JVector position, JMatrix matrix)
        public static JVector Transform(JVector position, JMatrix matrix)
        {
            JVector.Transform(position, matrix, out JVector result);
            return result;
        }

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <param name="result">The transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in JVector position, in JMatrix matrix, out JVector result)
        {
            double num0 = ((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31);
            double num1 = ((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32);
            double num2 = ((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33);

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        /// <summary>
        /// Transforms a vector by the transposed of the given Matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <param name="result">The transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransposedTransform(in JVector position, in JMatrix matrix, out JVector result)
        {
            double num0 = ((position.X * matrix.M11) + (position.Y * matrix.M12)) + (position.Z * matrix.M13);
            double num1 = ((position.X * matrix.M21) + (position.Y * matrix.M22)) + (position.Z * matrix.M23);
            double num2 = ((position.X * matrix.M31) + (position.Y * matrix.M32)) + (position.Z * matrix.M33);

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }
        #endregion

        /// <summary>
        /// Calculates the dot product of both vectors.
        /// </summary>
        /// <param name="vector1">The first vector.</param>
        /// <param name="vector2">The second vector.</param>
        /// <returns>Returns the dot product of both vectors.</returns>
        #region public static double Dot(JVector vector1, JVector vector2)

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(in JVector vector1, in JVector vector2)
        {
            return ((vector1.X * vector2.X) + (vector1.Y * vector2.Y)) + (vector1.Z * vector2.Z);
        }
        #endregion

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The sum of both vectors.</returns>
        #region public static void Add(JVector value1, JVector value2)
        public static JVector Add(in JVector value1, in JVector value2)
        {
            JVector.Add(value1, value2, out JVector result);
            return result;
        }

        /// <summary>
        /// Adds to vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <param name="result">The sum of both vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in JVector value1, in JVector value2, out JVector result)
        {
            result.X = value1.X + value2.X;
            result.Y = value1.Y + value2.Y;
            result.Z = value1.Z + value2.Z;
        }
        #endregion

        /// <summary>
        /// Subtracts to vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <param name="result">The difference of both vectors.</param>
        #region public static JVector Subtract(JVector value1, JVector value2)

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in JVector value1, in JVector value2, out JVector result)
        {
            result.X = value1.X - value2.X;
            result.Y = value1.Y - value2.Y;
            result.Z = value1.Z - value2.Z;
        }
        #endregion

        /// <summary>
        /// The cross product of two vectors.
        /// </summary>
        /// <param name="vector1">The first vector.</param>
        /// <param name="vector2">The second vector.</param>
        /// <returns>The cross product of both vectors.</returns>
        #region public static JVector Cross(JVector vector1, JVector vector2)
        public static JVector Cross(in JVector vector1, in JVector vector2)
        {
            JVector.Cross(vector1, vector2, out JVector result);
            return result;
        }

        /// <summary>
        /// The cross product of two vectors.
        /// </summary>
        /// <param name="vector1">The first vector.</param>
        /// <param name="vector2">The second vector.</param>
        /// <param name="result">The cross product of both vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(in JVector vector1, in JVector vector2, out JVector result)
        {
            double num3 = (vector1.Y * vector2.Z) - (vector1.Z * vector2.Y);
            double num2 = (vector1.Z * vector2.X) - (vector1.X * vector2.Z);
            double num = (vector1.X * vector2.Y) - (vector1.Y * vector2.X);
            result.X = num3;
            result.Y = num2;
            result.Z = num;
        }
        #endregion

        /// <summary>
        /// Gets the hashcode of the vector.
        /// </summary>
        /// <returns>Returns the hashcode of the vector.</returns>
        #region public override int GetHashCode()
        public override int GetHashCode()
        {
            return X.GetHashCode() ^ Y.GetHashCode() ^ Z.GetHashCode();
        }
        #endregion

        /// <summary>
        /// Inverses the direction of the vector.
        /// </summary>
        #region public static JVector Negate(JVector value)
        public void Negate()
        {
            this.X = -this.X;
            this.Y = -this.Y;
            this.Z = -this.Z;
        }

        /// <summary>
        /// Inverses the direction of a vector.
        /// </summary>
        /// <param name="value">The vector to inverse.</param>
        /// <returns>The negated vector.</returns>
        public static JVector Negate(in JVector value)
        {
            JVector.Negate(value, out JVector result);
            return result;
        }

        /// <summary>
        /// Inverses the direction of a vector.
        /// </summary>
        /// <param name="value">The vector to inverse.</param>
        /// <param name="result">The negated vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in JVector value, out JVector result)
        {
            double num0 = -value.X;
            double num1 = -value.Y;
            double num2 = -value.Z;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }
        #endregion

        /// <summary>
        /// Normalizes the given vector.
        /// </summary>
        /// <param name="value">The vector which should be normalized.</param>
        /// <returns>A normalized vector.</returns>
        #region public static JVector Normalize(JVector value)
        public static JVector Normalize(in JVector value)
        {
            JVector.Normalize(value, out JVector result);
            return result;
        }

        /// <summary>
        /// Normalizes this vector.
        /// </summary>
        public void Normalize()
        {
            double num2 = ((this.X * this.X) + (this.Y * this.Y)) + (this.Z * this.Z);
            double num = 1f / ((double)Math.Sqrt((double)num2));
            this.X *= num;
            this.Y *= num;
            this.Z *= num;
        }

        /// <summary>
        /// Normalizes the given vector.
        /// </summary>
        /// <param name="value">The vector which should be normalized.</param>
        /// <param name="result">A normalized vector.</param>
        public static void Normalize(in JVector value, out JVector result)
        {
            double num2 = ((value.X * value.X) + (value.Y * value.Y)) + (value.Z * value.Z);
            double num = 1f / ((double)Math.Sqrt((double)num2));
            result.X = value.X * num;
            result.Y = value.Y * num;
            result.Z = value.Z * num;
        }
        #endregion

        /// <summary>
        /// Gets the squared length of the vector.
        /// </summary>
        /// <returns>Returns the squared length of the vector.</returns>
        #region public double LengthSquared()
        public double LengthSquared()
        {
            return (((this.X * this.X) + (this.Y * this.Y)) + (this.Z * this.Z));
        }
        #endregion

        /// <summary>
        /// Gets the length of the vector.
        /// </summary>
        /// <returns>Returns the length of the vector.</returns>
        #region public double Length()
        public double Length()
        {
            double num = ((this.X * this.X) + (this.Y * this.Y)) + (this.Z * this.Z);
            return (double)Math.Sqrt((double)num);
        }
        #endregion

        #region public static void Swap(ref JVector vector1, ref JVector vector2)

        /// <summary>
        /// Swaps the components of both vectors.
        /// </summary>
        /// <param name="vector1">The first vector to swap with the second.</param>
        /// <param name="vector2">The second vector to swap with the first.</param>
        public static void Swap(ref JVector vector1, ref JVector vector2)
        {
            double temp;

            temp = vector1.X;
            vector1.X = vector2.X;
            vector2.X = temp;

            temp = vector1.Y;
            vector1.Y = vector2.Y;
            vector2.Y = temp;

            temp = vector1.Z;
            vector1.Z = vector2.Z;
            vector2.Z = temp;
        }
        #endregion

        /// <summary>
        /// Multiply a vector with a factor.
        /// </summary>
        /// <param name="value1">The vector to multiply.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <returns>Returns the multiplied vector.</returns>
        #region public static JVector Multiply(JVector value1, double scaleFactor)
        public static JVector Multiply(in JVector value1, double scaleFactor)
        {
            JVector.Multiply(value1, scaleFactor, out JVector result);
            return result;
        }

        /// <summary>
        /// Multiply a vector with a factor.
        /// </summary>
        /// <param name="value1">The vector to multiply.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <param name="result">Returns the multiplied vector.</param>
        public static void Multiply(in JVector value1, double scaleFactor, out JVector result)
        {
            result.X = value1.X * scaleFactor;
            result.Y = value1.Y * scaleFactor;
            result.Z = value1.Z * scaleFactor;
        }
        #endregion

        /// <summary>
        /// Calculates the cross product of two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>Returns the cross product of both.</returns>
        #region public static JVector operator %(JVector value1, JVector value2)
        public static JVector operator %(in JVector value1, in JVector value2)
        {
            JVector.Cross(value1, value2, out JVector result);
            return result;
        }
        #endregion

        /// <summary>
        /// Calculates the dot product of two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>Returns the dot product of both.</returns>
        #region public static double operator *(JVector value1, JVector value2)
        public static double operator *(in JVector value1, in JVector value2)
        {
            return JVector.Dot(value1, value2);
        }
        #endregion


        public static JVector operator /(in JVector value1, double value2)
        {
            JVector.Multiply(value1, 1.0d / value2, out JVector result);
            return result;
        }

        /// <summary>
        /// Multiplies a vector by a scale factor.
        /// </summary>
        /// <param name="value1">The vector to scale.</param>
        /// <param name="value2">The scale factor.</param>
        /// <returns>Returns the scaled vector.</returns>
        #region public static JVector operator *(JVector value1, double value2)
        public static JVector operator *(in JVector value1, double value2)
        {
            JVector.Multiply(value1, value2, out JVector result);
            return result;
        }
        #endregion

        /// <summary>
        /// Multiplies a vector by a scale factor.
        /// </summary>
        /// <param name="value2">The vector to scale.</param>
        /// <param name="value1">The scale factor.</param>
        /// <returns>Returns the scaled vector.</returns>
        #region public static JVector operator *(double value1, JVector value2)
        public static JVector operator *(double value1, in JVector value2)
        {
            JVector.Multiply(value2, value1, out JVector result);
            return result;
        }
        #endregion

        /// <summary>
        /// Subtracts two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The difference of both vectors.</returns>
        #region public static JVector operator -(JVector value1, JVector value2)
        public static JVector operator -(JVector value1, JVector value2)
        {
            JVector.Subtract(value1, value2, out JVector result);
            return result;
        }
        #endregion

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The sum of both vectors.</returns>
        #region public static JVector operator +(JVector value1, JVector value2)
        public static JVector operator +(in JVector value1, in JVector value2)
        {
            JVector.Add(value1, value2, out JVector result);
            return result;
        }
        #endregion
    }
}
