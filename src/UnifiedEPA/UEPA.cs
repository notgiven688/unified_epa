// UnifiedEPA
// (c) Thorben Linneweber
// SPDX-License-Identifier: MIT
//
// Robustness notes
// ----------------
// This solver starts from a tiny interior tetrahedron around an interior point of the
// Minkowski difference and expands a triangulated hull from there.
//
// In the separating case, the hull can contain very flat triangles. That is acceptable in
// practice because progress is driven mainly by the current closest point to the origin,
// not by uniformly well-conditioned faces everywhere in the hull.
//
// The barycentric computation clamps the projected point back to the triangle whenever the
// projection falls outside. For very flat but still nondegenerate triangles, this often
// makes the effective closest feature one of the triangle's edges. The next search direction
// is then taken from that clamped closest point, so the algorithm can continue to make
// useful progress even when local triangles are poorly conditioned.
//
// Truly degenerate triangles are rejected during creation. The method therefore does not
// rely on zero-area triangles; it relies on the fact that poorly shaped but valid triangles
// still provide useful closest-point information in the separating case.
using System;

namespace UEPADemo
{
    public sealed class UEPA
    {
        // Very small epsilon for geometric degeneracy checks.
        private const double NumericEpsilon = 1e-24d;

        // Convergence threshold for the support-gap test.
        private const double CollideEpsilon = 1e-6d;

        // Hard iteration limit.
        private const int MaxIter = 84;

        public struct Statistics
        {
            public double Accuracy;
            public int Iterations;
        }

        public class UEPASolver
        {
            public Statistics Statistics;
            public MinkowskiDifference MKD;

            public struct Triangle
            {
                public short A, B, C;

                // Used by the enclosure test.
                public bool FacingOrigin;

                public short this[int i] => i switch
                {
                    0 => A,
                    1 => B,
                    _ => C
                };

                // Cached face normal and closest point to the origin.
                public JVector Normal;
                public JVector ClosestToOrigin;

                public double NormalSq;
                public double ClosestToOriginSq;
            }

            public struct Edge(short a, short b)
            {
                public short A = a;
                public short B = b;

                // Horizon extraction treats edges as undirected.
                public static bool Equals(in Edge a, in Edge b)
                {
                    return (a.A == b.A && a.B == b.B) || (a.A == b.B && a.B == b.A);
                }
            }

            // Euler characteristic for a closed triangulated convex hull:
            //   V - E + F = 2
            // With only triangles and every edge shared by two triangles:
            //   F = 2 * V - 4
            private const int MaxVertices = MaxIter + 4;
            private const int MaxTriangles = 2 * MaxVertices;

            private readonly Triangle[] triangles = new Triangle[MaxTriangles];
            private readonly JVector[] vertices = new JVector[MaxVertices];
            private readonly JVector[] verticesA = new JVector[MaxVertices];
            private readonly JVector[] verticesB = new JVector[MaxVertices];

            // Temporary storage for horizon edges while rebuilding the hull.
            private readonly Edge[] edges = new Edge[MaxVertices * 3 / 2];

            private short vPointer = 0;
            private short tCount = 0;

            private bool originEnclosed = false;
            private JVector center;

            public bool CalcBarycentric(in Triangle tri, out JVector result)
            {
                JVector a = vertices[tri.A];
                JVector b = vertices[tri.B];
                JVector c = vertices[tri.C];

                bool clamped = false;

                // Calculate the barycentric coordinates of the origin projected
                // onto the plane of the triangle.
                //
                // [W. Heidrich, Journal of Graphics, GPU, and Game Tools,
                //  Volume 10, Issue 3, 2005.]

                JVector u, v, w, tmp;
                JVector.Subtract(a, b, out u);
                JVector.Subtract(a, c, out v);

                double t = tri.NormalSq;
                JVector.Cross(u, a, out tmp);
                double gamma = JVector.Dot(tmp, tri.Normal) / t;
                JVector.Cross(a, v, out tmp);
                double beta = JVector.Dot(tmp, tri.Normal) / t;
                double alpha = 1.0d - gamma - beta;

                // Clamp the projected barycentric coordinates to the triangle
                // so they describe the closest point on the triangle itself.
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

                result.X = alpha;
                result.Y = beta;
                result.Z = gamma;
                return clamped;
            }

            // Small regular tetrahedron around the interior point returned by SupportCenter.
            private const double SeedScale = 1e-4d;

            private readonly JVector seed0 = SeedScale * new JVector(Math.Sqrt(8.0d / 9.0d), 0.0d, -1.0d / 3.0d);
            private readonly JVector seed1 = SeedScale * new JVector(-Math.Sqrt(2.0d / 9.0d), Math.Sqrt(2.0d / 3.0d), -1.0d / 3.0d);
            private readonly JVector seed2 = SeedScale * new JVector(-Math.Sqrt(2.0d / 9.0d), -Math.Sqrt(2.0d / 3.0d), -1.0d / 3.0d);
            private readonly JVector seed3 = SeedScale * new JVector(0.0d, 0.0d, 1.0d);

            private void ConstructInitialTetrahedron()
            {
                vPointer = 3;

                vertices[0] = center + seed0;
                vertices[1] = center + seed1;
                vertices[2] = center + seed2;
                vertices[3] = center + seed3;

                // Tetrahedron faces wound so the hull encloses "center".
                CreateTriangle(0, 2, 1);
                CreateTriangle(0, 1, 3);
                CreateTriangle(0, 3, 2);
                CreateTriangle(1, 2, 3);
            }

            private bool IsLit(int candidate, int w)
            {
                ref Triangle tri = ref triangles[candidate];
                JVector deltaA = vertices[w] - vertices[tri.A];
                return JVector.Dot(deltaA, tri.Normal) > 0;
            }

            private bool CreateTriangle(short a, short b, short c)
            {
                ref Triangle triangle = ref triangles[tCount];
                triangle.A = a;
                triangle.B = b;
                triangle.C = c;

                if (!TryInitializeTriangle(ref triangle))
                    return false;

                tCount++;
                return true;
            }

            private bool TryInitializeTriangle(ref Triangle triangle)
            {
                JVector.Subtract(vertices[triangle.A], vertices[triangle.B], out JVector u);
                JVector.Subtract(vertices[triangle.A], vertices[triangle.C], out JVector v);
                JVector.Cross(u, v, out triangle.Normal);
                triangle.NormalSq = triangle.Normal.LengthSquared();

                // Degenerate triangles do not contribute useful hull geometry.
                if (triangle.NormalSq < NumericEpsilon)
                    return false;

                // Orient the face so the seed center stays inside the hull.
                double delta = JVector.Dot(triangle.Normal, vertices[triangle.A] - center);

                if (delta < 0)
                {
                    (triangle.A, triangle.B) = (triangle.B, triangle.A);
                    triangle.Normal.Negate();
                }

                // Used by the origin enclosure test.
                delta = JVector.Dot(triangle.Normal, vertices[triangle.A]);
                triangle.FacingOrigin = delta > 0.0d;

                // Cache the closest point from this face to the origin.
                if (CalcBarycentric(triangle, out JVector bc))
                {
                    triangle.ClosestToOrigin =
                        bc.X * vertices[triangle.A] +
                        bc.Y * vertices[triangle.B] +
                        bc.Z * vertices[triangle.C];

                    triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
                }
                else
                {
                    // The origin projects inside the triangle.
                    JVector.Multiply(triangle.Normal, delta / triangle.NormalSq, out triangle.ClosestToOrigin);
                    triangle.ClosestToOriginSq = triangle.ClosestToOrigin.LengthSquared();
                }

                return true;
            }

            private int FindClosestTriangleAndUpdateEnclosure()
            {
                int closestIndex = -1;
                double currentMin = double.MaxValue;

                // Once originEnclosed becomes true, it stays true for subsequent iterations.
                bool latchedEnclosure = originEnclosed;
                originEnclosed = true;

                for (int i = 0; i < tCount; i++)
                {
                    if (triangles[i].ClosestToOriginSq < currentMin)
                    {
                        currentMin = triangles[i].ClosestToOriginSq;
                        closestIndex = i;
                    }

                    if (!triangles[i].FacingOrigin)
                        originEnclosed = latchedEnclosure;
                }

                return closestIndex;
            }

            private short AddSupportVertex(in JVector direction)
            {
                vPointer++;
                MKD.Support(direction, out verticesA[vPointer], out verticesB[vPointer], out vertices[vPointer]);
                return vPointer;
            }

            private void AddOrRemoveEdge(in Edge edge, ref int edgeCount)
            {
                for (int i = edgeCount - 1; i >= 0; i--)
                {
                    if (Edge.Equals(edges[i], edge))
                    {
                        edges[i] = edges[--edgeCount];
                        return;
                    }
                }

                edges[edgeCount++] = edge;
            }

            private int RemoveLitTrianglesAndCollectBoundary(short newVertex)
            {
                int edgeCount = 0;

                for (int index = tCount - 1; index >= 0; index--)
                {
                    if (!IsLit(index, newVertex))
                        continue;

                    // Shared interior edges cancel out, horizon edges remain.
                    for (int k = 0; k < 3; k++)
                    {
                        Edge edge = new Edge(triangles[index][k], triangles[index][(k + 1) % 3]);
                        AddOrRemoveEdge(edge, ref edgeCount);
                    }

                    // Remove the visible face by swapping in the last active face.
                    triangles[index] = triangles[--tCount];
                }

                return edgeCount;
            }

            private bool FinalizeResult(in Triangle closestTriangle, double deltaDist, bool originEnclosed,
                out JVector point1, out JVector point2, out JVector normal, out double separation)
            {
                separation = Math.Sqrt(closestTriangle.ClosestToOriginSq);

                // Negative separation denotes penetration.
                if (originEnclosed)
                    separation *= -1.0d;

                Statistics.Accuracy = Math.Abs(deltaDist / separation);

                CalcBarycentric(closestTriangle, out JVector bc);

                point1 =
                    bc.X * verticesA[closestTriangle.A] +
                    bc.Y * verticesA[closestTriangle.B] +
                    bc.Z * verticesA[closestTriangle.C];

                point2 =
                    bc.X * verticesB[closestTriangle.A] +
                    bc.Y * verticesB[closestTriangle.B] +
                    bc.Z * verticesB[closestTriangle.C];

                // In the common case, the normal comes from the closest point on the
                // Minkowski hull. If that point collapses to zero, use the face normal.
                if (Math.Abs(separation) > NumericEpsilon)
                {
                    normal = -closestTriangle.ClosestToOrigin * (1.0f / separation);
                }
                else
                {
                    normal = closestTriangle.Normal * (1.0f / Math.Sqrt(closestTriangle.NormalSq));
                }

                return true;
            }

            public bool Solve(out JVector point1, out JVector point2, out JVector normal, out double separation)
            {
                tCount = 0;
                originEnclosed = false;

                // Seed the hull around an interior point of the Minkowski difference.
                MKD.SupportCenter(out center);
                ConstructInitialTetrahedron();

                for (int iter = 1; iter < MaxIter; iter++)
                {
                    Statistics.Iterations = iter;

                    // Pick the face whose closest point to the origin is minimal.
                    int closestIndex = FindClosestTriangleAndUpdateEnclosure();
                    Triangle closestTriangle = triangles[closestIndex];

                    // In the separating case, search opposite the current closest point.
                    // In the penetrating case, search outward from the hull.
                    JVector searchDir = closestTriangle.ClosestToOrigin;
                    double searchDirSq = closestTriangle.ClosestToOriginSq;

                    if (!originEnclosed)
                        searchDir.Negate();

                    // If the closest point has collapsed to the origin, use the face normal.
                    if (closestTriangle.ClosestToOriginSq < NumericEpsilon)
                    {
                        searchDir = closestTriangle.Normal;
                        searchDirSq = closestTriangle.NormalSq;
                    }

                    short newVertex = AddSupportVertex(searchDir);

                    // Termination test:
                    // Can the new support point still extend the hull meaningfully
                    // along the current search direction?
                    double deltaDist = JVector.Dot(closestTriangle.ClosestToOrigin - vertices[newVertex], searchDir);

                    if (deltaDist * deltaDist <= CollideEpsilon * CollideEpsilon * searchDirSq)
                    {
                        return FinalizeResult(closestTriangle, deltaDist, originEnclosed,
                            out point1, out point2, out normal, out separation);
                    }

                    // Remove visible faces and collect the boundary of the cavity.
                    int edgeCount = RemoveLitTrianglesAndCollectBoundary(newVertex);

                    // Stitch the new support point into the hull along the horizon.
                    for (int i = 0; i < edgeCount; i++)
                    {
                        if (!CreateTriangle(edges[i].A, edges[i].B, newVertex))
                        {
                            return FinalizeResult(closestTriangle, deltaDist, originEnclosed,
                                out point1, out point2, out normal, out separation);
                        }
                    }

                    // No horizon means the hull can no longer be extended meaningfully.
                    if (edgeCount == 0)
                    {
                        return FinalizeResult(closestTriangle, deltaDist, originEnclosed,
                            out point1, out point2, out normal, out separation);
                    }
                }

                point1 = point2 = normal = JVector.Zero;
                separation = 0.0d;

                System.Diagnostics.Debug.WriteLine($"EPA: Could not converge within {MaxIter} iterations.");
                return false;
            }
        }

        [ThreadStatic] public static UEPASolver epaSolver;

        public static bool Detect(ISupportMappable supportA, ISupportMappable supportB,
            in JMatrix orientationA, in JMatrix orientationB, in JVector positionA, in JVector positionB,
            out JVector pointA, out JVector pointB, out double separation)
        {
            epaSolver ??= new UEPASolver();

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