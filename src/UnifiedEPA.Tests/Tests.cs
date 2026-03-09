// UnifiedEPA
// (c) Thorben Linneweber
// SPDX-License-Identifier: MIT
using System;
using UEPADemo;

namespace UnifiedEPA.Tests
{
    public class UEPATests
    {
        [Fact]
        public void CubeSphereStress()
        {
            for (int i = 0; i < 100000; i++) CubeSphere(i);
        }

        [Fact]
        public void SphereSphereAnalyticalCheck()
        {
            for (int i = 0; i < 10000; i++) SphereSphere(i);
        }

        [Fact]
        public void CubeCubeAnalyticalCheck()
        {
            for (int i = 0; i < 100000; i++) CubeCube(i);
        }

        private static void CubeCube(int i)
        {
            var s1 = new CubeShape();
            var s2 = new CubeShape();

            JMatrix rot1 = JMatrix.Identity;
            JMatrix rot2 = JMatrix.Identity;

            JVector pos1 = new JVector(1.0d + (double)i / 10.0d, 0.1d, 0.0d);
            JVector pos2 = new JVector(-(double)i / 10.0d, 0.0d, 0.0d);

            UEPA.Detect(s1, s2, rot1, rot2, pos1, pos2,
                out _, out _, out double separation);

            double analyticalDistance = 2.0d * (double)i / 10.0d;

            Assert.True(Math.Abs(analyticalDistance - separation) <= 1e-5d,
                $"Distance {separation} does not match analytical result {analyticalDistance}.");
        }

        private static void CubeSphere(int i)
        {
            var s1 = new CubeShape();
            var s2 = new SphereShape();

            JMatrix rot1 = JMatrix.CreateRotationX((double)i);
            JMatrix rot2 = JMatrix.CreateRotationX(-0.7d * (double)i);

            JVector pos1 = new JVector(0.1d, 0.1d, 0.2d);
            JVector pos2 = new JVector(0.8d, 0.3d, 0.4d);

            UEPA.Detect(s1, s2, rot1, rot2, pos1, pos2,
                out _, out _, out _);
        }

        private static void SphereSphere(int i)
        {
            var s1 = new SphereShape();
            var s2 = new SphereShape();

            JMatrix rot1 = JMatrix.CreateRotationX((double)i);
            JMatrix rot2 = JMatrix.CreateRotationY(-(double)i);

            JVector pos1 = new JVector(0.1d + (double)i / 1e5d, 0.1d, 0.2d);
            JVector pos2 = new JVector(0.8d, 0.3d, 0.4d);

            UEPA.Detect(s1, s2, rot1, rot2, pos1, pos2,
                out _, out _, out double separation);

            double analyticalDistance = (pos2 - pos1).Length() - 1.0d;

            Assert.True(Math.Abs(analyticalDistance - separation) <= 1e-5d,
                $"Distance {separation} does not match analytical result {analyticalDistance}.");
        }
    }
}
