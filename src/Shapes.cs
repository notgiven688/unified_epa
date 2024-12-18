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
using System.Reflection;
using System.Collections.Generic;

namespace UEPADemo
{
    public class ShapeAttribute : Attribute
    {
        public string Name { get; private set; }
        public ShapeAttribute(string name) => this.Name = name;

        public static IEnumerable<Type> GetAllTypes()
        {
            Assembly assembly = Assembly.GetExecutingAssembly();
            foreach (Type type in assembly.GetTypes())
            {
                if (type.GetCustomAttributes(typeof(ShapeAttribute), true).Length > 0)
                    yield return type;
            }
        }
    }

    public interface ISupportMappable
    {
        void SupportMapping(in JVector direction, out JVector result);
    }

    public static class ShapeHelper
    {
        public static void SupportLine(in JVector direction, out JVector result)
        {
            JVector a = new JVector(0, 0.5d, 0);
            JVector b = new JVector(0, -0.5d, 0);

            double t0 = JVector.Dot(direction, a);
            double t2 = JVector.Dot(direction, b);

            if (t0 > t2) result = a;
            else result = b;
        }

        public static void SupportTriangle(in JVector direction, out JVector result)
        {
            JVector a = new JVector(0, 0, 1);
            JVector b = new JVector(-1, 0, -1);
            JVector c = new JVector(1, 0, -1);

            double t0 = JVector.Dot(direction, a);
            double t1 = JVector.Dot(direction, b);
            double t2 = JVector.Dot(direction, c);

            if (t0 > t1) result = t0 > t2 ? a : c;
            else result = t2 > t1 ? c : b;
        }

        public static void SupportDisc(in JVector direction, out JVector result)
        {
            result.X = direction.X;
            result.Y = 0;
            result.Z = direction.Z;

            if (result.LengthSquared() > 1e-12) result.Normalize();
            result *= 0.5d;
        }

        public static void SupportSphere(in JVector direction, out JVector result)
        {
            result = direction;
            result.Normalize();
        }

        public static void SupportCone(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportDisc(direction, out JVector res1);
            JVector res2 = new JVector(0, 1, 0);

            if (JVector.Dot(direction, res1) >= JVector.Dot(direction, res2)) result = res1;
            else result = res2;

            result.Y -= 0.5d;
        }

        public static void SupportCube(in JVector direction, out JVector result)
        {
            result.X = Math.Sign(direction.X);
            result.Y = Math.Sign(direction.Y);
            result.Z = Math.Sign(direction.Z);
        }
    }

    [Shape("Cylinder")]
    public class CylinderShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportDisc(direction, out JVector res1);
            ShapeHelper.SupportLine(direction, out JVector res2);
            result = res1 + res2;
        }
    }

    [Shape("Cube")]
    public class CubeShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportCube(direction, out result);
            result *= 0.5d;
        }
    }

    [Shape("Line Segment")]
    public class LineShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportLine(direction, out JVector res1);
            ShapeHelper.SupportSphere(direction, out JVector res2);
            result = res1 + res2 * 0.01d;
        }
    }

    [Shape("Triangle")]
    public class TriangleShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportTriangle(direction, out JVector res1);
            ShapeHelper.SupportSphere(direction, out JVector res2);
            result = res1 + res2 * 0.01d;
        }
    }

    [Shape("Sphere")]
    public class SphereShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportSphere(direction, out result);
            result *= 0.5d;
        }
    }

    [Shape("Ellipsoid")]
    public class EllipsoidShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            // ellipsoid == affine transformation of a sphere
            JVector dir = direction;
            dir.X *= 0.5f; dir.Y *= 0.8f; dir.Z *= 0.2f;
            ShapeHelper.SupportSphere(dir, out result);
            result.X *= 0.5f; result.Y *= 0.8f; result.Z *= 0.2f;
        }
    }

    [Shape("Capped Cone")]
    public class CappedConeShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportDisc(direction, out JVector res1);
            ShapeHelper.SupportCone(direction, out JVector res2);
            result = (res1 + res2) * 0.5d;
        }
    }

    [Shape("Capsule")]
    public class CapsuleShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportLine(direction, out JVector res1);
            ShapeHelper.SupportSphere(direction, out JVector res2);
            result = (res1 + res2 * 0.5d) * 0.8d;
        }
    }

    [Shape("Cone")]
    public class ConeShape : ISupportMappable
    {
        public void SupportMapping(in JVector direction, out JVector result)
        {
            ShapeHelper.SupportCone(direction, out result);
        }

    }
}
