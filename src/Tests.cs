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

namespace GJKEPADemo
{
    public static class Tests
    {
        public static void RunTests()
        {
            RunTest("cube/sphere stress", () => { for (int i = 1; i < 100000; i++) CubeSphere(i); });
            RunTest("sphere/sphere analytical check", () => { for (int i = 1; i < 10000; i++) SphereSphere(i); });
            RunTest("cube/cube analytical check", () => { for (int i = 1; i < 100000; i++) CubeCube(i); });
        }

        public static void RunTest(string desc, Action test)
        {
            Console.WriteLine(new string('-', 60));
            Console.WriteLine($"Running Test \"{desc}\"..");
            var sw = System.Diagnostics.Stopwatch.StartNew();
            test();
            Console.WriteLine($"Test \"{desc}\" took {sw.ElapsedMilliseconds}ms.");
        }

        private static void CubeCube(int i)
        {
            var s1 = new CubeShape();
            var s2 = new CubeShape();

            JMatrix rot1 = JMatrix.Identity;
            JMatrix rot2 = JMatrix.Identity;

            JVector pos1 = new JVector(1.0d + (double)i / 10.0d, 0.1d, 0.0d);
            JVector pos2 = new JVector(-(double)i / 10.0d, 0.0d, 0.0d);

            JVector p1, p2;
            double separation;

            GJKEPA.Detect(s1, s2, ref rot1, ref rot2, ref pos1, ref pos2,
            out p1, out p2, out separation);

            double analyticalDistance = 2.0d * (double)i / 10.0d;

            if (Math.Abs(analyticalDistance - separation) > 1e-5d)
                throw new Exception("Distance does not match analytical result.");
        }

        private static void CubeSphere(int i)
        {
            var s1 = new CubeShape();
            var s2 = new SphereShape();

            JMatrix rot1 = JMatrix.CreateRotationX((double)i);
            JMatrix rot2 = JMatrix.CreateRotationX(-0.7d * (double)i);

            JVector pos1 = new JVector(0.1d, 0.1d, 0.2d);
            JVector pos2 = new JVector(0.8d, 0.3d, 0.4d);

            JVector p1, p2;
            double separation;

            GJKEPA.Detect(s1, s2, ref rot1, ref rot2, ref pos1, ref pos2,
            out p1, out p2, out separation);
        }

        private static void SphereSphere(int i)
        {
            var s1 = new SphereShape();
            var s2 = new SphereShape();

            JMatrix rot1 = JMatrix.CreateRotationX((double)i);
            JMatrix rot2 = JMatrix.CreateRotationY(-(double)i);

            JVector pos1 = new JVector(0.1d + (double)i / 1e5d, 0.1d, 0.2d);
            JVector pos2 = new JVector(0.8d, 0.3d, 0.4d);

            JVector p1, p2;
            double separation;

            GJKEPA.Detect(s1, s2, ref rot1, ref rot2, ref pos1, ref pos2,
            out p1, out p2, out separation);

            double analyticalDistance = (pos2 - pos1).Length() - 1.0d;

            if (Math.Abs(analyticalDistance - separation) > 1e-5d)
                throw new Exception("Distance does not match analytical result.");
        }
    }
}
