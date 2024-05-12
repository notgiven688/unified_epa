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
    public sealed class GJKEPA
    {
        private const double NumericEpsilon = 1e-24d;
        private const double CollideEpsilon = 1e-6d;
        private const int MaxIter = 84;

        public struct Statistics { public double Accuracy; public int Iterations; }

        public struct MinkowskiDifference
        {
            public ISupportMappable SupportA, SupportB;
            public JMatrix OrientationA, OrientationB;
            public JVector PositionA, PositionB;

            private void SupportMapTransformedA(in JVector direction, out JVector result)
            {
                JVector.TransposedTransform(direction, OrientationA, out JVector tmp);
                SupportA.SupportMapping(tmp, out result);
                JVector.Transform(result, OrientationA, out result);
                JVector.Add(result, PositionA, out result);
            }

            private void SupportMapTransformedB(in JVector direction, out JVector result)
            {
                JVector.TransposedTransform(direction, OrientationB, out JVector tmp);
                SupportB.SupportMapping(tmp, out result);
                JVector.Transform(result, OrientationB, out result);
                JVector.Add(result, PositionB, out result);
            }

            public void Support(in JVector direction, out JVector vA, out JVector vB, out JVector v)
            {
                JVector.Negate(direction, out JVector tmp);
                SupportMapTransformedA(tmp, out vA);
                SupportMapTransformedB(direction, out vB);
                JVector.Subtract(vA, vB, out v);
            }

            public void SupportMapping(in JVector direction, out JVector result)
            {
                this.Support(direction, out _, out _, out result);
            }

            public void SupportCenter(out JVector center)
            {
                JVector.Subtract(PositionA, PositionB, out center);
            }
        }

        public class GJKEPASolver
        {
            public Statistics Statistics;

            public MinkowskiDifference MKD;

            public struct Triangle
            {
                public short A, B, C;
                public bool FacingOrigin;

                public short this[int i]
                {
                    get { return i == 0 ? A : (i == 1 ? B : C); }
                }

                public JVector Normal;
                public JVector ClosestToOrigin;

                public double NormalSq;
                public double ClosestToOriginSq;
            }

            public struct Edge
            {
                public short A;
                public short B;

                public Edge(short a, short b)
                {
                    this.A = a;
                    this.B = b;
                }

                public static bool Equals(in Edge a, in Edge b)
                {
                    return ((a.A == b.A && a.B == b.B) || (a.A == b.B && a.B == b.A));
                }
            }

            // (*) Euler-characteristic: V (vertices) - E (edges) + F (faces) = 2
            // We have triangles T instead of faces: F = T
            // and every edge shares two triangles -> T = 2*V - 4
            private const int MaxVertices = MaxIter + 4;
            private const int MaxTriangles = 2 * MaxVertices;

            private readonly Triangle[] Triangles = new Triangle[MaxTriangles];
            private readonly JVector[] Vertices = new JVector[MaxVertices];
            private readonly JVector[] VerticesA = new JVector[MaxVertices];
            private readonly JVector[] VerticesB = new JVector[MaxVertices];

            // see (*)
            private readonly Edge[] edges = new Edge[MaxVertices * 3 / 2];

            private short vPointer = 0;
            private short tCount = 0;

            private bool originEnclosed = false;
            private JVector center;

            public bool CalcBarycentric(in Triangle tri, out JVector result)
            {
                JVector a = Vertices[tri.A];
                JVector b = Vertices[tri.B];
                JVector c = Vertices[tri.C];

                bool clamped = false;

                // Calculate the barycentric coordinates of the origin (0,0,0) projected
                // onto the plane of the triangle.
                //
                // [W. Heidrich, Journal of Graphics, GPU, and Game Tools,Volume 10, Issue 3, 2005.]

                JVector u, v, w, tmp;
                JVector.Subtract(a, b, out u);
                JVector.Subtract(a, c, out v);

                double t = tri.NormalSq;
                JVector.Cross(u, a, out tmp);
                double gamma = JVector.Dot(tmp, tri.Normal) / t;
                JVector.Cross(a, v, out tmp);
                double beta = JVector.Dot(tmp, tri.Normal) / t;
                double alpha = 1.0d - gamma - beta;

                // Clamp the projected barycentric coordinates to lie within the triangle,
                // such that the clamped coordinates are closest (euclidean) to the original point.
                //
                // [https://math.stackexchange.com/questions/
                //  1092912/find-closest-point-in-triangle-given-barycentric-coordinates-outside]

                if (alpha >= 0.0d && beta < 0.0d)
                {
                    t = JVector.Dot(a, u);
                    if ((gamma < 0.0d) && (t > 0.0d))
                    {
                        beta = Math.Min(1.0d, t / u.LengthSquared());
                        alpha = 1.0d - beta;
                        gamma = 0.0d;
                    }
                    else
                    {
                        gamma = Math.Min(1.0d, Math.Max(0.0d, JVector.Dot(a, v) / v.LengthSquared()));
                        alpha = 1.0d - gamma;
                        beta = 0.0d;
                    }

                    clamped = true;
                }
                else if (beta >= 0.0d && gamma < 0.0d)
                {
                    JVector.Subtract(b, c, out w);
                    t = JVector.Dot(b, w);
                    if ((alpha < 0.0d) && (t > 0.0d))
                    {
                        gamma = Math.Min(1.0d, t / w.LengthSquared());
                        beta = 1.0d - gamma;
                        alpha = 0.0d;
                    }
                    else
                    {
                        alpha = Math.Min(1.0d, Math.Max(0.0d, -JVector.Dot(b, u) / u.LengthSquared()));
                        beta = 1.0d - alpha;
                        gamma = 0.0d;
                    }

                    clamped = true;
                }
                else if (gamma >= 0.0d && alpha < 0.0d)
                {
                    JVector.Subtract(b, c, out w);
                    t = -JVector.Dot(c, v);
                    if ((beta < 0.0d) && (t > 0.0d))
                    {
                        alpha = Math.Min(1.0d, t / v.LengthSquared());
                        gamma = 1.0d - alpha;
                        beta = 0.0d;
                    }
                    else
                    {
                        beta = Math.Min(1.0d, Math.Max(0.0d, -JVector.Dot(c, w) / w.LengthSquared()));
                        gamma = 1.0d - beta;
                        alpha = 0.0d;
                    }

                    clamped = true;
                }

                result.X = alpha; result.Y = beta; result.Z = gamma;
                return clamped;
            }

            const double scale = 1e-4d;
            readonly JVector v0 = scale * new JVector(Math.Sqrt(8.0d / 9.0d), 0.0d, -1.0d / 3.0d);
            readonly JVector v1 = scale * new JVector(-Math.Sqrt(2.0d / 9.0d), Math.Sqrt(2.0d / 3.0d), -1.0d / 3.0d);
            readonly JVector v2 = scale * new JVector(-Math.Sqrt(2.0d / 9.0d), -Math.Sqrt(2.0d / 3.0d), -1.0d / 3.0d);
            readonly JVector v3 = scale * new JVector(0.0d, 0.0d, 1.0d);

            private void ConstructInitialTetrahedron()
            {
                vPointer = 3;

                Vertices[0] = v0 + center;
                Vertices[1] = v1 + center;
                Vertices[2] = v2 + center;
                Vertices[3] = v3 + center;

                CreateTriangle(0, 2, 1);
                CreateTriangle(0, 1, 3);
                CreateTriangle(0, 3, 2);
                CreateTriangle(1, 2, 3);
            }

            private bool IsLit(int candidate, int w)
            {
                ref Triangle tr = ref Triangles[candidate];
                JVector deltaA = Vertices[w] - Vertices[tr.A];
                return JVector.Dot(deltaA, tr.Normal) > 0;
            }

            private bool CreateTriangle(short a, short b, short c)
            {
                ref Triangle triangle = ref Triangles[tCount];
                triangle.A = a; triangle.B = b; triangle.C = c;

                JVector.Subtract(Vertices[a], Vertices[b], out JVector u);
                JVector.Subtract(Vertices[a], Vertices[c], out JVector v);
                JVector.Cross(u, v, out triangle.Normal);
                triangle.NormalSq = triangle.Normal.LengthSquared();

                // no need to add degenerate triangles
                if (triangle.NormalSq < NumericEpsilon) return false;

                // do we need to flip the triangle? (the origin of the md has to be enclosed)
                double delta = JVector.Dot(triangle.Normal, Vertices[a] - center);

                if (delta < 0)
                {
                    (triangle.A, triangle.B) = (triangle.B, triangle.A);
                    triangle.Normal.Negate();
                }

                delta = JVector.Dot(triangle.Normal, Vertices[a]);
                triangle.FacingOrigin = delta > 0.0d;

                if (CalcBarycentric(triangle, out JVector bc))
                {
                    triangle.ClosestToOrigin = bc.X * Vertices[triangle.A] + bc.Y * Vertices[triangle.B] + bc.Z * Vertices[triangle.C];
                    triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
                }
                else
                {
                    // Prefer point-plane distance calculation if possible.
                    JVector.Multiply(triangle.Normal, delta / triangle.NormalSq, out triangle.ClosestToOrigin);
                    triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
                }

                tCount++;
                return true;
            }

            public bool Solve(out JVector point1, out JVector point2, out JVector normal, out double separation)
            {
                tCount = 0;
                originEnclosed = false;

                MKD.SupportCenter(out center);
                ConstructInitialTetrahedron();

                int iter = 0;
                Triangle ctri; // closest Triangle

                while (++iter < MaxIter)
                {
                    this.Statistics.Iterations = iter;

                    // search for the closest triangle and check if the origin is enclosed
                    int closestIndex = -1;
                    double currentMin = double.MaxValue;
                    bool skipTest = originEnclosed;

                    originEnclosed = true;

                    for (int i = 0; i < tCount; i++)
                    {
                        if(Triangles[i].ClosestToOriginSq < currentMin)
                        {
                            currentMin = Triangles[i].ClosestToOriginSq;
                            closestIndex = i;
                        }

                        if(!Triangles[i].FacingOrigin) originEnclosed = skipTest;
                    }

                    ctri = Triangles[closestIndex];

                    JVector searchDir = ctri.ClosestToOrigin;
                    double searchDirSq = ctri.ClosestToOriginSq;

                    if (originEnclosed) searchDir.Negate();

                    if(ctri.ClosestToOriginSq < NumericEpsilon)
                    {
                        searchDir = ctri.Normal;
                        searchDirSq = ctri.NormalSq;
                    }

                    vPointer++;
                    MKD.Support(searchDir, out VerticesA[vPointer], out VerticesB[vPointer], out Vertices[vPointer]);

                    // Termination condition
                    // Can we further "extend" the convex hull by adding the new vertex?
                    //
                    // v = Vertices[vPointer] (support point)
                    // c = Triangles[Head].ClosestToOrigin
                    // s = searchDir
                    //
                    // abs(dot(c - v, s)) / len(s) < e <=> [dot(c - v, s)]^2 < e*e*s^2
                    double deltaDist = JVector.Dot(ctri.ClosestToOrigin - Vertices[vPointer], searchDir);

                    if (deltaDist * deltaDist <= CollideEpsilon * CollideEpsilon * searchDirSq)
                    {
                        goto converged;
                    }

                    int ePointer = 0;
                    for (int index = tCount; index-- > 0;)
                    {
                        if (!IsLit(index, vPointer)) continue;
                        Edge edge; bool added;

                        for (int k = 0; k < 3; k++)
                        {
                            edge = new (Triangles[index][(k + 0) % 3], Triangles[index][(k + 1) % 3]);
                            added = true;
                            for (int e = ePointer; e-- > 0;)
                            {
                                if (Edge.Equals(edges[e], edge))
                                {
                                    edges[e] = edges[--ePointer];
                                    added = false;
                                }
                            }
                            if (added) edges[ePointer++] = edge;
                        }
                        Triangles[index] = Triangles[--tCount];
                    }

                    for (int i = 0; i < ePointer; i++)
                    {
                        if(!CreateTriangle(edges[i].A, edges[i].B, vPointer))
                            goto converged;
                    }

                    if (ePointer > 0) continue;

converged:
                    separation = Math.Sqrt(ctri.ClosestToOriginSq);
                    if(originEnclosed) separation *= -1.0d;

                    this.Statistics.Accuracy = Math.Abs(deltaDist / separation);

                    CalcBarycentric(ctri, out JVector bc);

                    point1 = bc.X * VerticesA[ctri.A] + bc.Y * VerticesA[ctri.B] + bc.Z * VerticesA[ctri.C];
                    point2 = bc.X * VerticesB[ctri.A] + bc.Y * VerticesB[ctri.B] + bc.Z * VerticesB[ctri.C];

                    normal = ctri.Normal * (1.0d / Math.Sqrt(ctri.NormalSq));

                    return true;
                }

                point1 = point2 = normal = JVector.Zero;
                separation = 0.0d;

                System.Diagnostics.Debug.WriteLine($"EPA: Could not converge within {MaxIter} iterations.");

                return false;
            }
        }

        [ThreadStatic]
        public static GJKEPASolver epaSolver;

        public static bool Detect(ISupportMappable supportA, ISupportMappable supportB,
         in JMatrix orientationA, in JMatrix orientationB,
         in JVector positionA, in JVector positionB,
         out JVector pointA, out JVector pointB, out double separation)
        {
            epaSolver ??= new GJKEPASolver();

            epaSolver.MKD.SupportA = supportA;
            epaSolver.MKD.SupportB = supportB;
            epaSolver.MKD.OrientationA = orientationA;
            epaSolver.MKD.OrientationB = orientationB;
            epaSolver.MKD.PositionA = positionA;
            epaSolver.MKD.PositionB = positionB;

            bool success = epaSolver.Solve(out pointA, out pointB, out _, out separation);

            return success;
        }
    }
}
