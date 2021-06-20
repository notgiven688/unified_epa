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
        private const double CollideEpsilon = 1e-6d;
        private const double NumericEpsilon = 1e-12d;
        
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
                public double SqDistance;

                public JVector Normal, NNormal;
                public JVector SearchDirection;

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

                    if (Triangles[triangle].SqDistance < Triangles[current].SqDistance)
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

            public void CalcPoint(int triangle, ref JVector barycentric, out JVector result)
            {
                ref Triangle t = ref Triangles[triangle];
                JMatrix m = new JMatrix(ref Vertices[t.A], ref Vertices[t.B], ref Vertices[t.C]);
                JVector.Transform(ref barycentric, ref m, out result);
            }

            public void CalcPointA(int triangle, ref JVector barycentric, out JVector result)
            {
                ref Triangle t = ref Triangles[triangle];
                JMatrix m = new JMatrix(ref VerticesA[t.A], ref VerticesA[t.B], ref VerticesA[t.C]);
                JVector.Transform(ref barycentric, ref m, out result);
            }

            public void CalcPointB(int triangle, ref JVector barycentric, out JVector result)
            {
                ref Triangle t = ref Triangles[triangle];
                JMatrix m = new JMatrix(ref VerticesB[t.A], ref VerticesB[t.B], ref VerticesB[t.C]);
                JVector.Transform(ref barycentric, ref m, out result);
            }

            public void CalcBarycentric(int triangle, bool project, out JVector result)
            {
                Triangle tri = Triangles[triangle];
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

                double t = tri.Normal.LengthSquared();
                JVector.Cross(ref u, ref a, out tmp);
                double gamma = JVector.Dot(ref tmp, ref tri.Normal) / t;
                JVector.Cross(ref a, ref v, out tmp);
                double beta = JVector.Dot(ref tmp, ref tri.Normal) / t;
                double alpha = 1.0d - gamma - beta;

                if (project)
                {
                    result.X = alpha; result.Y = beta; result.Z = gamma;
                    return;
                }

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
                        gamma = Math.Min(1, t / w.LengthSquared());
                        beta = 1.0d - gamma;
                        alpha = 0.0d;
                    }
                    else
                    {
                        alpha = Math.Min(1, Math.Max(0.0d, -JVector.Dot(ref b, ref u) / u.LengthSquared()));
                        beta = 1.0d - alpha;
                        gamma = 0.0d;
                    }
                }
                else if (gamma >= 0.0d && alpha < 0.0d)
                {
                    JVector.Subtract(ref b, ref c, out w);
                    t = -JVector.Dot(ref c, ref v);
                    if ((beta < 0) && (t > 0.0d))
                    {
                        alpha = Math.Min(1, t / v.LengthSquared());
                        gamma = 1.0d - alpha;
                        beta = 0.0d;
                    }
                    else
                    {
                        beta = Math.Min(1, Math.Max(0, -JVector.Dot(ref c, ref w) / w.LengthSquared()));
                        gamma = 1.0d - beta;
                        alpha = 0.0d;
                    }
                }

                result.X = alpha; result.Y = beta; result.Z = gamma;
            }

            public int CreateTriangle(int a, int b, int c)
            {
                ref Triangle triangle = ref Triangles[tPointer];
                triangle.A = a; triangle.B = b; triangle.C = c;
                triangle.Next = triangle.Prev = -1;

                JVector u, v;
                JVector.Subtract(ref Vertices[a], ref Vertices[b], out u);
                JVector.Subtract(ref Vertices[a], ref Vertices[c], out v);

                JVector.Cross(ref u, ref v, out triangle.Normal);
                JVector.Normalize(ref triangle.Normal, out triangle.NNormal);

                if (OriginEnclosed)
                {
                    triangle.SearchDirection = triangle.NNormal;
                    triangle.SqDistance = JVector.Dot(ref Vertices[a], ref triangle.NNormal);
                    triangle.SqDistance *= triangle.SqDistance;
                }
                else
                {
                    CalcBarycentric(tPointer, false, out JVector bc);
                    CalcPoint(tPointer, ref bc, out triangle.SearchDirection);
                    triangle.SqDistance = triangle.SearchDirection.LengthSquared();
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

            public void ConstructInitialTetrahedron(JVector position)
            {
                vPointer = 3;

                Vertices[0] = v0;
                Vertices[1] = v1;
                Vertices[2] = v2;
                Vertices[3] = v3;

                JVector.Add(ref Vertices[0], ref position, out Vertices[0]);
                JVector.Add(ref Vertices[1], ref position, out Vertices[1]);
                JVector.Add(ref Vertices[2], ref position, out Vertices[2]);
                JVector.Add(ref Vertices[3], ref position, out Vertices[3]);

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
                tPointer = 0;
                vPointer = 0;
                OriginEnclosed = false;
                Head = -1;

                point1 = point2 = JVector.Zero;
                separation = 0.0d;

                double distToOriginSq = 0;
                int counter = 0;

                MKD.SupportCenter(out JVector center);
                ConstructInitialTetrahedron(center);

                while (++counter < MaxIter)
                {
                    this.Statistics.Iterations = counter;

                    JVector v = Triangles[Head].SearchDirection;

                    if (OriginEnclosed) v.Negate();

                    vPointer++;
                    MKD.Support(ref v, out VerticesA[vPointer], out VerticesB[vPointer], out Vertices[vPointer]);

                    int closest = -1;

                    for (int node = Head; node != -1; node = Triangles[node].Next)
                    {
                        if (IsLit(node, vPointer))
                        {
                            closest = node;
                            break;
                        }
                    }

                    if (closest == -1)
                    {
                        System.Diagnostics.Debug.WriteLine(
                            "exit reason 1 - new point not clearly outside polytope.");

                        return false;
                    }

                    if (OriginEnclosed)
                    {
                        // EPA termination condition: The closest face of the polytope
                        // can not be extended further than CollideEpsilon.

                        JVector.Subtract(ref Vertices[vPointer], ref Vertices[Triangles[closest].A], out JVector delta);
                        double deltaDist = Math.Abs(JVector.Dot(ref Triangles[closest].NNormal, ref delta));

                        if (deltaDist < CollideEpsilon)
                        {
                            this.Statistics.Accuracy = deltaDist;
                            ref Triangle s = ref Triangles[closest];
                            separation = -Math.Sqrt(s.SqDistance);
                            CalcBarycentric(closest, true, out JVector bc);
                            CalcPointA(closest, ref bc, out point1);
                            CalcPointB(closest, ref bc, out point2);
                            return true;
                        }
                    }

                    ExpandHorizon(closest, vPointer);
                    System.Diagnostics.Debug.Assert(ntPointer > 0);

                    if (!OriginEnclosed)
                    {
                        int i = 0;
                        for (; i < ntPointer; i++)
                        {
                            ref Triangle t = ref Triangles[newTriangles[i]];
                            double dd = JVector.Dot(ref t.NNormal, ref Vertices[t.A]);
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
                                "exit reason 2 - horizon incomplete.");

                            return false;
                        }

                        Triangles[lastIndex].SetN3(newTriangles[found]);
                        Triangles[newTriangles[found]].SetN1(lastIndex);

                        lastIndex = newTriangles[found];
                        newTriangles[found] = newTriangles[--ntPointer];
                    }

                    Triangles[firstIndex].SetN1(lastIndex);
                    Triangles[lastIndex].SetN3(firstIndex);

                    if (!OriginEnclosed)
                    {
                        // GJK termination condition: Successive results for the closest 
                        // distance of the polytope and the origin fall under the threeshold
                        // of CollideEpsilon.

                        double x = Triangles[Head].SqDistance - distToOriginSq;
                        distToOriginSq = Triangles[Head].SqDistance;
                        double dd2 = (x * x) / (4.0d * distToOriginSq);

                        if (dd2 < CollideEpsilon * CollideEpsilon)
                        {
                            this.Statistics.Accuracy = Math.Sqrt(dd2);
                            ref Triangle s = ref Triangles[Head];
                            separation = Math.Sqrt(s.SqDistance);
                            CalcBarycentric(Head, false, out JVector bc);
                            CalcPointA(Head, ref bc, out point1);
                            CalcPointB(Head, ref bc, out point2);
                            return true;
                        }
                    }
                }

                Diagnostics.Debug.WriteLine(
                    $"exit reason 3: could not converge within {MaxIter} iterations.");

                return false;
            }

            private bool IsLit(int candidate, int w)
            {
                ref Triangle tr = ref Triangles[candidate];
                JVector.Subtract(ref Vertices[w], ref Vertices[tr.A], out JVector deltaA);
                return (JVector.Dot(ref deltaA, ref tr.NNormal) > -NumericEpsilon);
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

        public static bool Detect(ISupportMappable support1, ISupportMappable support2,
         ref JMatrix orientation1, ref JMatrix orientation2,
         ref JVector position1, ref JVector position2,
         out JVector point1, out JVector point2, out double separation)
        {
            if (epaSolver == null) epaSolver = new GJKEPASolver();

            epaSolver.MKD.SupportA = support1;
            epaSolver.MKD.SupportB = support2;
            epaSolver.MKD.OrientationA = orientation1;
            epaSolver.MKD.OrientationB = orientation2;
            epaSolver.MKD.PositionA = position1;
            epaSolver.MKD.PositionB = position2;

            bool success = epaSolver.Solve(out point1, out point2, out separation);

            return success;
        }

    }

}
