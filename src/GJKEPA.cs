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
using Diagnostics = System.Diagnostics;

namespace GJKEPADemo
{
    public sealed class GJKEPA
    {
        private const double NumericEpsilon = 1e-12d;
        private const double CollideEpsilon = 1e-6d;
        private const int MaxIter = 85;

        public struct Statistics { public double Accuracy; public int Iterations; }

        public class MinkowskiDifference : ISupportMappable
        {
            public ISupportMappable SupportA, SupportB;
            public JMatrix OrientationA, OrientationB;
            public JVector PositionA, PositionB;

            private void SupportMapTransformedA(ref JVector direction, out JVector result)
            {
                JVector.TransposedTransform(ref direction, ref OrientationA, out JVector tmp);
                SupportA.SupportMapping(ref tmp, out result);
                JVector.Transform(ref result, ref OrientationA, out result);
                JVector.Add(ref result, ref PositionA, out result);
            }

            private void SupportMapTransformedB(ref JVector direction, out JVector result)
            {
                JVector.TransposedTransform(ref direction, ref OrientationB, out JVector tmp);
                SupportB.SupportMapping(ref tmp, out result);
                JVector.Transform(ref result, ref OrientationB, out result);
                JVector.Add(ref result, ref PositionB, out result);
            }

            public void Support(ref JVector direction, out JVector vA, out JVector vB, out JVector v)
            {
                JVector.Negate(ref direction, out JVector tmp);
                SupportMapTransformedA(ref tmp, out vA);
                SupportMapTransformedB(ref direction, out vB);
                JVector.Subtract(ref vA, ref vB, out v);
            }

            public void SupportMapping(ref JVector direction, out JVector result)
            {
                this.Support(ref direction, out _, out _, out result);
            }

            public void SupportCenter(out JVector center)
            {
                JVector.Subtract(ref PositionA, ref PositionB, out center);
            }
        }

        public class GJKEPASolver
        {
            public Statistics Statistics;

            public struct Triangle
            {
                public int A, B, C;
                public int N1, N2, N3;

                public int Prev, Next;

                public bool Visited;
                
                public JVector Normal;
                public JVector ClosestToOrigin;

                public double NormalSq;
                public double ClosestToOriginSq;

                public void SetNeighbors(int n1, int n2, int n3)
                {
                    this.N1 = n1; this.N2 = n2; this.N3 = n3;
                }

                public void SetN1(int n1) => this.N1 = n1;
                public void SetN2(int n2) => this.N2 = n2;
                public void SetN3(int n3) => this.N3 = n3;
            }

            public MinkowskiDifference MKD = new MinkowskiDifference();

            private Triangle[] Triangles = new Triangle[1024];
            private int Head = -1;

            private JVector[] Vertices = new JVector[512];
            private JVector[] VerticesA = new JVector[512];
            private JVector[] VerticesB = new JVector[512];

            private int tPointer = 0;
            private int vPointer = 0;

            bool OriginEnclosed = false;

            int[] newTriangles = new int[512];
            int ntPointer = 0;

            private void RemoveTriangle(int triangle)
            {
                int prev = Triangles[triangle].Prev;
                int next = Triangles[triangle].Next;

                if(prev != -1) Triangles[prev].Next = next;
                else Head = next;

                if(next != -1) Triangles[next].Prev = prev;
            }

            private void SortInTriangle(int triangle)
            {
                int current = -1;

                for(int next = Head; next != -1; next = Triangles[current].Next)
                {
                    current = next;

                    if (Triangles[triangle].ClosestToOriginSq < Triangles[current].ClosestToOriginSq)
                    {
                        int prev = Triangles[current].Prev;
                        Triangles[triangle].Next = current;
                        Triangles[triangle].Prev = prev;
                        Triangles[current].Prev = triangle;
                        if(prev != -1) Triangles[prev].Next = triangle;
                        else Head = triangle;
                        return;
                    }
                }

                Triangles[triangle].Prev = current;
                if(current != -1) Triangles[current].Next = triangle;
                else Head = triangle;
            }

            private void CalcPoint(int triangle, ref JVector barycentric, out JVector result)
            {
                ref Triangle t = ref Triangles[triangle];
                JMatrix m = new JMatrix(ref Vertices[t.A], ref Vertices[t.B], ref Vertices[t.C]);
                JVector.Transform(ref barycentric, ref m, out result);
            }

            private void CalcPointA(int triangle, ref JVector barycentric, out JVector result)
            {
                ref Triangle t = ref Triangles[triangle];
                JMatrix m = new JMatrix(ref VerticesA[t.A], ref VerticesA[t.B], ref VerticesA[t.C]);
                JVector.Transform(ref barycentric, ref m, out result);
            }

            private void CalcPointB(int triangle, ref JVector barycentric, out JVector result)
            {
                ref Triangle t = ref Triangles[triangle];
                JMatrix m = new JMatrix(ref VerticesB[t.A], ref VerticesB[t.B], ref VerticesB[t.C]);
                JVector.Transform(ref barycentric, ref m, out result);
            }

            public void CalcBarycentric(int triangle, out JVector result)
            {
                ref Triangle tri = ref Triangles[triangle];
                JVector a = Vertices[tri.A];
                JVector b = Vertices[tri.B];
                JVector c = Vertices[tri.C];

                JVector u, v, tmp;
                JVector.Subtract(ref a, ref b, out u);
                JVector.Subtract(ref a, ref c, out v);

                double t = tri.NormalSq;
                JVector.Cross(ref u, ref a, out tmp);
                double gamma = JVector.Dot(ref tmp, ref tri.Normal) / t;
                JVector.Cross(ref a, ref v, out tmp);
                double beta = JVector.Dot(ref tmp, ref tri.Normal) / t;
                double alpha = 1.0d - gamma - beta;

                result.X = alpha; result.Y = beta; result.Z = gamma;
            }

            public void CalcBarycentricProject(int triangle, out JVector result)
            {
                ref Triangle tri = ref Triangles[triangle];
                JVector a = Vertices[tri.A];
                JVector b = Vertices[tri.B];
                JVector c = Vertices[tri.C];

                // Calculate the barycentric coordinates of the origin (0,0,0) projected
                // onto the plane of the triangle.
                // 
                // [W. Heidrich, Journal of Graphics, GPU, and Game Tools,Volume 10, Issue 3, 2005.]

                JVector u, v, w, tmp;
                JVector.Subtract(ref a, ref b, out u);
                JVector.Subtract(ref a, ref c, out v);

                double t = tri.NormalSq;
                JVector.Cross(ref u, ref a, out tmp);
                double gamma = JVector.Dot(ref tmp, ref tri.Normal) / t;
                JVector.Cross(ref a, ref v, out tmp);
                double beta = JVector.Dot(ref tmp, ref tri.Normal) / t;
                double alpha = 1.0d - gamma - beta;

                // Clamp the projected barycentric coordinates to lie within the triangle,
                // such that the clamped coordinates are closest (euclidean) to the original point.
                //
                // [https://math.stackexchange.com/questions/
                //  1092912/find-closest-point-in-triangle-given-barycentric-coordinates-outside]

                if (alpha >= 0.0d && beta < 0.0d)
                {
                    t = JVector.Dot(ref a, ref u);
                    if ((gamma < 0.0d) && (t > 0.0d))
                    {
                        beta = Math.Min(1.0d, t / u.LengthSquared());
                        alpha = 1.0d - beta;
                        gamma = 0.0d;
                    }
                    else
                    {
                        gamma = Math.Min(1.0d, Math.Max(0.0d, JVector.Dot(ref a, ref v) / v.LengthSquared()));
                        alpha = 1.0d - gamma;
                        beta = 0.0d;
                    }
                }
                else if (beta >= 0.0d && gamma < 0.0d)
                {
                    JVector.Subtract(ref b, ref c, out w);
                    t = JVector.Dot(ref b, ref w);
                    if ((alpha < 0.0d) && (t > 0.0d))
                    {
                        gamma = Math.Min(1.0d, t / w.LengthSquared());
                        beta = 1.0d - gamma;
                        alpha = 0.0d;
                    }
                    else
                    {
                        alpha = Math.Min(1.0d, Math.Max(0.0d, -JVector.Dot(ref b, ref u) / u.LengthSquared()));
                        beta = 1.0d - alpha;
                        gamma = 0.0d;
                    }
                }
                else if (gamma >= 0.0d && alpha < 0.0d)
                {
                    JVector.Subtract(ref b, ref c, out w);
                    t = -JVector.Dot(ref c, ref v);
                    if ((beta < 0.0d) && (t > 0.0d))
                    {
                        alpha = Math.Min(1.0d, t / v.LengthSquared());
                        gamma = 1.0d - alpha;
                        beta = 0.0d;
                    }
                    else
                    {
                        beta = Math.Min(1.0d, Math.Max(0.0d, -JVector.Dot(ref c, ref w) / w.LengthSquared()));
                        gamma = 1.0d - beta;
                        alpha = 0.0d;
                    }
                }

                result.X = alpha; result.Y = beta; result.Z = gamma;
            }

            private int CreateTriangle(int a, int b, int c)
            {
                ref Triangle triangle = ref Triangles[tPointer];
                triangle.A = a; triangle.B = b; triangle.C = c;
                triangle.Next = triangle.Prev = -1;

                JVector u, v;
                JVector.Subtract(ref Vertices[a], ref Vertices[b], out u);
                JVector.Subtract(ref Vertices[a], ref Vertices[c], out v);
                JVector.Cross(ref u, ref v, out triangle.Normal);
                triangle.NormalSq = triangle.Normal.LengthSquared();

                if (triangle.NormalSq < NumericEpsilon * NumericEpsilon)
                {
                    triangle.ClosestToOrigin = JVector.Zero;
                    triangle.ClosestToOriginSq = float.NaN;
                }
                else
                {
                    if(OriginEnclosed)
                    {
                        double delta = JVector.Dot(ref triangle.Normal,ref Vertices[a]);
                        JVector.Multiply(ref triangle.Normal, delta / triangle.NormalSq, out triangle.ClosestToOrigin);
                        triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
                    }
                    else
                    {
                        CalcBarycentricProject(tPointer, out JVector bc);
                        CalcPoint(tPointer, ref bc, out triangle.ClosestToOrigin);
                        triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
                    }
                }
                
                triangle.Visited = false;
                SortInTriangle(tPointer);

                return tPointer++;
            }

            const double scale = 1e-4d;
            JVector v0 = scale * new JVector(Math.Sqrt(8.0d / 9.0d), 0.0d, -1.0d / 3.0d);
            JVector v1 = scale * new JVector(-Math.Sqrt(2.0d / 9.0d), Math.Sqrt(2.0d / 3.0d), -1.0d / 3.0d);
            JVector v2 = scale * new JVector(-Math.Sqrt(2.0d / 9.0d), -Math.Sqrt(2.0d / 3.0d), -1.0d / 3.0d);
            JVector v3 = scale * new JVector(0.0d, 0.0d, 1.0d);

            private void ConstructInitialTetrahedron(JVector position)
            {
                vPointer = 3;

                JVector.Add(ref v0, ref position, out Vertices[0]);
                JVector.Add(ref v1, ref position, out Vertices[1]);
                JVector.Add(ref v2, ref position, out Vertices[2]);
                JVector.Add(ref v3, ref position, out Vertices[3]);

                int t1 = CreateTriangle(0, 2, 1);
                int t2 = CreateTriangle(0, 1, 3);
                int t3 = CreateTriangle(0, 3, 2);
                int t4 = CreateTriangle(1, 2, 3);

                Triangles[t1].SetNeighbors(t2, t3, t4);
                Triangles[t2].SetNeighbors(t3, t1, t4);
                Triangles[t3].SetNeighbors(t1, t2, t4);
                Triangles[t4].SetNeighbors(t2, t1, t3);
            }

            public bool Solve(out JVector point1, out JVector point2, out double separation)
            {
                tPointer = vPointer = ntPointer = 0;
                OriginEnclosed = false;
                Head = -1;

                point1 = point2 = JVector.Zero;
                separation = 0.0d;

                int iter = 0;

                MKD.SupportCenter(out JVector center);
                ConstructInitialTetrahedron(center);

                while (++iter < MaxIter)
                {
                    this.Statistics.Iterations = iter;

                    JVector searchDir = Triangles[Head].ClosestToOrigin;
                    if (OriginEnclosed) searchDir.Negate();

                    vPointer++;
                    MKD.Support(ref searchDir, out VerticesA[vPointer], out VerticesB[vPointer], out Vertices[vPointer]);

                    // Search for a triangle on the existing polytope which is "lighted" by the new support point.
                    // The (double-linked) list of triangles is sorted by their distance to the origin. This allows
                    // for an efficient search.

                    int ltri = -1;

                    for (int node = Head; node != -1; node = Triangles[node].Next)
                    {
                        if (IsLit(node, vPointer))
                        {
                            ltri = node;
                            break;
                        }
                    }

                    // Termination condition for GJK and EPA
                    //     c = Triangles[Head].ClosestToOrigin (closest point on the polytope)
                    //     v = Vertices[vPointer] (support point)
                    //     e = CollideEpsilon
                    // The termination condition reads: 
                    //     abs(dot(normalize(c), v - c)) < e
                    //     <=>  abs(dot(c, v - c))/len(c) < e <=> abs((dot(c, v) - dot(c,c)))/len(c) < e
                    //     <=>  (dot(c, v) - dot(c,c))^2 < e^2*c^2 <=> (dot(c, v) - c^2)^2 < e^2*c^2

                    double deltaDist = Triangles[Head].ClosestToOriginSq - JVector.Dot(ref Vertices[vPointer], ref Triangles[Head].ClosestToOrigin);
                    bool tc = deltaDist * deltaDist < CollideEpsilon * CollideEpsilon * Triangles[Head].ClosestToOriginSq;

                    // Check if new support point is in the set of already found points.
                    // Compare with the detailed discussion in
                    // [Gino van den Bergen, Collision Detection in Interactive 3D Environments]

                    bool repeat = false;
                    for (int i = 4; i < vPointer; i++)
                    {
                        if(Math.Abs(Vertices[i].X - Vertices[vPointer].X) > NumericEpsilon) continue;
                        if(Math.Abs(Vertices[i].Y - Vertices[vPointer].Y) > NumericEpsilon) continue;
                        if(Math.Abs(Vertices[i].Z - Vertices[vPointer].Z) > NumericEpsilon) continue;

                        repeat = true;
                        break;
                    }

                    if (ltri == -1 || repeat || tc)
                    {
                        separation = Math.Sqrt(Triangles[Head].ClosestToOriginSq);
                        this.Statistics.Accuracy = Math.Abs(deltaDist) / separation;

                        if (OriginEnclosed) separation *= -1;

                        JVector bc;
                        if (OriginEnclosed) CalcBarycentric(Head, out bc);
                        else CalcBarycentricProject(Head, out bc);

                        CalcPointA(Head, ref bc, out point1);
                        CalcPointB(Head, ref bc, out point2);
                        return true;
                    }

                    // Adjacent triangles are searched (flood-fill algorithm) to determine all lighted triangles
                    // which then get removed. The hole in the polytope is filled using new triangles connecting 
                    // the "horizon" and the new support point. Indices of neighboring triangles have to be updated.

                    ExpandHorizon(ltri, vPointer);
                    System.Diagnostics.Debug.Assert(ntPointer > 0);

                    if (!OriginEnclosed)
                    {
                        int i = 0;
                        for (; i < ntPointer; i++)
                        {
                            ref Triangle t = ref Triangles[newTriangles[i]];
                            double dd = JVector.Dot(ref t.Normal, ref Vertices[t.A]);
                            if (dd < 0.0d) break;
                        }
                        OriginEnclosed = (i == ntPointer);
                    }

                    int firstIndex = newTriangles[0];
                    int lastIndex = firstIndex;

                    newTriangles[0] = newTriangles[--ntPointer];

                    while (ntPointer > 0)
                    {
                        int found = -1;
                        for (int i = 0; i < ntPointer; i++)
                        {
                            if (Triangles[newTriangles[i]].A == Triangles[lastIndex].B)
                            {
                                found = i;
                                break;
                            }
                        }

                        if (found == -1)
                        {
                            System.Diagnostics.Debug.WriteLine(
                                "exit reason 1 - horizon incomplete.");

                            return false;
                        }

                        Triangles[lastIndex].SetN3(newTriangles[found]);
                        Triangles[newTriangles[found]].SetN1(lastIndex);

                        lastIndex = newTriangles[found];
                        newTriangles[found] = newTriangles[--ntPointer];
                    }

                    Triangles[firstIndex].SetN1(lastIndex);
                    Triangles[lastIndex].SetN3(firstIndex);
                }

                Diagnostics.Debug.WriteLine(
                    $"exit reason 2: could not converge within {MaxIter} iterations.");

                return false;
            }

            private bool IsLit(int candidate, int w)
            {
                ref Triangle tr = ref Triangles[candidate];
                JVector.Subtract(ref Vertices[w], ref Vertices[tr.A], out JVector deltaA);
                return JVector.Dot(ref deltaA, ref tr.Normal) > 0;
            }

            private void ExpandHorizon(int candidate, int w)
            {
                ref Triangle tri = ref Triangles[candidate];

                if (tri.Visited) return;
                tri.Visited = true;

                RemoveTriangle(candidate);

                bool n1Lit = IsLit(tri.N1, w);
                bool n2Lit = IsLit(tri.N2, w);
                bool n3Lit = IsLit(tri.N3, w);

                ref Triangle n1 = ref Triangles[tri.N1];
                ref Triangle n2 = ref Triangles[tri.N2];
                ref Triangle n3 = ref Triangles[tri.N3];

                if (!n1Lit)
                {
                    int newIndex = CreateTriangle(tri.C, tri.A, w);
                    Triangles[newIndex].SetN2(tri.N1);
                    newTriangles[ntPointer++] = newIndex;

                    if (n1.N1 == candidate) n1.N1 = newIndex;
                    else if (n1.N2 == candidate) n1.N2 = newIndex;
                    else n1.N3 = newIndex;
                }

                if (!n2Lit)
                {
                    int newIndex = CreateTriangle(tri.A, tri.B, w);
                    Triangles[newIndex].SetN2(tri.N2);
                    newTriangles[ntPointer++] = newIndex;

                    if (n2.N1 == candidate) n2.N1 = newIndex;
                    else if (n2.N2 == candidate) n2.N2 = newIndex;
                    else n2.N3 = newIndex;
                }

                if (!n3Lit)
                {
                    int newIndex = CreateTriangle(tri.B, tri.C, w);
                    Triangles[newIndex].SetN2(tri.N3);
                    newTriangles[ntPointer++] = newIndex;

                    if (n3.N1 == candidate) n3.N1 = newIndex;
                    else if (n3.N2 == candidate) n3.N2 = newIndex;
                    else n3.N3 = newIndex;
                }

                if (n1Lit) ExpandHorizon(tri.N1, w);
                if (n2Lit) ExpandHorizon(tri.N2, w);
                if (n3Lit) ExpandHorizon(tri.N3, w);
            }
        }

        [ThreadStatic]
        public static GJKEPASolver epaSolver;

        public static bool Detect(ISupportMappable supportA, ISupportMappable supportB,
         ref JMatrix orientationA, ref JMatrix orientationB,
         ref JVector positionA, ref JVector positionB,
         out JVector pointA, out JVector pointB, out double separation)
        {
            if (epaSolver == null) epaSolver = new GJKEPASolver();

            epaSolver.MKD.SupportA = supportA;
            epaSolver.MKD.SupportB = supportB;
            epaSolver.MKD.OrientationA = orientationA;
            epaSolver.MKD.OrientationB = orientationB;
            epaSolver.MKD.PositionA = positionA;
            epaSolver.MKD.PositionB = positionB;

            bool success = epaSolver.Solve(out pointA, out pointB, out separation);

            return success;
        }
    }
}
