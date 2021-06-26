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

namespace GJKEPADemo
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
        void SupportMapping(ref JVector direction, out JVector result);
    }

    public static class ShapeHelper
    {
        public static void SupportLine(ref JVector direction, out JVector result)
        {
            JVector a = new JVector(0, 0.5d, 0);
            JVector b = new JVector(0, -0.5d, 0);

            double t0 = JVector.Dot(direction, a);
            double t2 = JVector.Dot(direction, b);

            if (t0 > t2) result = a;
            else result = b;
        }

        public static void SupportTriangle(ref JVector direction, out JVector result)
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

        public static void SupportDisc(ref JVector direction, out JVector result)
        {
            result.X = direction.X;
            result.Y = 0;
            result.Z = direction.Z;

            if (result.LengthSquared() > 1e-12) result.Normalize();
            result *= 0.5d;
        }

        public static void SupportSphere(ref JVector direction, out JVector result)
        {
            result = direction;
            result.Normalize();
        }

        public static void SupportCone(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportDisc(ref direction, out JVector res1);
            JVector res2 = new JVector(0, 1, 0);

            if (JVector.Dot(ref direction, ref res1) >= JVector.Dot(ref direction, ref res2)) result = res1;
            else result = res2;

            result.Y -= 0.5d;
        }

        public static void SupportCube(ref JVector direction, out JVector result)
        {
            result.X = Math.Sign(direction.X);
            result.Y = Math.Sign(direction.Y);
            result.Z = Math.Sign(direction.Z);
        }
    }

    [Shape("Cylinder")]
    public class CylinderShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportDisc(ref direction, out JVector res1);
            ShapeHelper.SupportLine(ref direction, out JVector res2);
            result = res1 + res2;
        }
    }

    [Shape("Cube")]
    public class CubeShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportCube(ref direction, out result);
            result *= 0.5d;
        }
    }

    [Shape("LineShape")]
    public class LineShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportLine(ref direction, out JVector res1);
            ShapeHelper.SupportSphere(ref direction, out JVector res2);
            result = res1 + res2 * 0.01d;
        }
    }

    [Shape("Triangle")]
    public class TriangleShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportTriangle(ref direction, out JVector res1);
            ShapeHelper.SupportSphere(ref direction, out JVector res2);
            result = res1 + res2 * 0.01d;
        }
    }

    [Shape("Sphere")]
    public class SphereShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportSphere(ref direction, out result);
            result *= 0.5d;
        }
    }

    [Shape("Capped Cone")]
    public class CappedConeShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportDisc(ref direction, out JVector res1);
            ShapeHelper.SupportCone(ref direction, out JVector res2);
            result = (res1 + res2) * 0.5d;
        }
    }

    [Shape("Capsule")]
    public class CapsuleShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportLine(ref direction, out JVector res1);
            ShapeHelper.SupportSphere(ref direction, out JVector res2);
            result = (res1 + res2 * 0.5d) * 0.8d;
        }
    }

    [Shape("Cone")]
    public class ConeShape : ISupportMappable
    {
        public void SupportMapping(ref JVector direction, out JVector result)
        {
            ShapeHelper.SupportCone(ref direction, out result);
        }

    }
}
