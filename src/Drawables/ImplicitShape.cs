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

#define SMOOTHGROUPS

using System;
using System.Linq;
using System.Collections.Generic;

using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;

namespace GJKEPADemo
{
    public class ImplicitShape : Primitive
    {
        private PhongShader phong;

        public float Alpha { get; set; }
        public Vector3 Angles { set; get; }

        public ImplicitShape(ISupportMappable support)
        {
            this.SupportMap = support;
        }

        public ISupportMappable SupportMap { private set; get; }

        private struct ClipTriangle
        {
            public JVector v1;
            public JVector v2;
            public JVector v3;
            public int division;
        }

        private struct ATriangle
        {
            public AVertex[] Vertices;
        }

        private class AVertex
        {
            public JVector Position;
            public JVector Normal;

            public AVertex(JVector p, JVector n)
            {
                this.Position = p;
                this.Normal = n;
            }
        }

        private void MakeHull(int subdivisions, bool smoothGroups = true)
        {
            List<AVertex> avertices = new List<AVertex>();
            List<ATriangle> atriangles = new List<ATriangle>();

            Stack<ClipTriangle> sphereTesselation = new Stack<ClipTriangle>();
            double gr = (1.0d + Math.Sqrt(5.0d)) / 2.0d;

            JVector[] vertices = new JVector[12]
            {
                new JVector(0, +1, +gr), new JVector(0, -1, +gr), new JVector(0, +1, -gr), new JVector(0, -1, -gr),
                new JVector(+1, +gr, 0), new JVector(+1, -gr, 0), new JVector(-1, +gr, 0), new JVector(-1, -gr, 0),
                new JVector(+gr, 0, +1), new JVector(+gr, 0, -1), new JVector(-gr, 0, +1), new JVector(-gr, 0, -1),
            };

            int[,] indices = new int[20, 3]
            {
                { 1, 0, 10 }, { 0, 1, 8 }, { 0, 4, 6 }, { 4, 0, 8 }, { 0, 6, 10 }, { 5, 1, 7 }, { 1, 5, 8 }, { 7, 1, 10 },
                { 2, 3, 11 }, { 3, 2, 9 }, { 4, 2, 6 }, { 2, 4, 9 }, { 6, 2, 11 }, { 3, 5, 7 }, { 5, 3, 9 }, { 3, 7, 11 },
                { 4, 8, 9 }, { 8, 5, 9 }, { 10, 6, 11 }, { 7, 10, 11 }
            };

            for (int i = 0; i < 20; i++)
            {
                ClipTriangle tri = new ClipTriangle();
                tri.v1 = vertices[indices[i, 0]];
                tri.v2 = vertices[indices[i, 1]];
                tri.v3 = vertices[indices[i, 2]];
                tri.division = 0;
                sphereTesselation.Push(tri);
            }

            while (sphereTesselation.Count > 0)
            {
                ClipTriangle tri = sphereTesselation.Pop();

                if (tri.division < subdivisions)
                {
                    ClipTriangle tri1, tri2, tri3, tri4;
                    JVector n;

                    tri1.division = tri.division + 1;
                    tri2.division = tri.division + 1;
                    tri3.division = tri.division + 1;
                    tri4.division = tri.division + 1;

                    tri1.v1 = tri.v1;
                    tri2.v2 = tri.v2;
                    tri3.v3 = tri.v3;

                    n = (tri.v1 + tri.v2) / 2.0d;
                    tri1.v2 = n; tri2.v1 = n; tri4.v3 = n;

                    n = (tri.v2 + tri.v3) / 2.0d;
                    tri2.v3 = n; tri3.v2 = n; tri4.v1 = n;

                    n = (tri.v3 + tri.v1) / 2.0d;
                    tri1.v3 = n; tri3.v1 = n; tri4.v2 = n;

                    sphereTesselation.Push(tri1);
                    sphereTesselation.Push(tri2);
                    sphereTesselation.Push(tri3);
                    sphereTesselation.Push(tri4);
                }
                else
                {
                    JVector p1; SupportMap.SupportMapping(ref tri.v1, out p1);
                    JVector p2; SupportMap.SupportMapping(ref tri.v2, out p2);
                    JVector p3; SupportMap.SupportMapping(ref tri.v3, out p3);
                    JVector n = (p3 - p1) % (p2 - p1); 

                    if (n.LengthSquared() > 1e-24d)
                    {
                        AVertex av1 = new AVertex(p1, n);
                        AVertex av2 = new AVertex(p2, n);
                        AVertex av3 = new AVertex(p3, n);

                        avertices.Add(av1);
                        avertices.Add(av2);
                        avertices.Add(av3);

                        ATriangle atri = new ATriangle();
                        atri.Vertices = new AVertex[3] { av1, av2, av3 };
                        atriangles.Add(atri);
                    }
                }
            }

#if SMOOTHGROUPS
            if (smoothGroups)
            {
                foreach (var group in avertices.GroupBy(s => s.Position))
                    SmoothGroup(new Stack<AVertex>(group));
            }
#endif

            int counter = 0;

            foreach (ATriangle atri in atriangles)
            {
                for (int i = 0; i < 3; i++)
                {
                    JVector p = atri.Vertices[i].Position;
                    JVector n = atri.Vertices[i].Normal;

                    AddIndex(counter++);

                    AddVertex(
                        new Vector3((float)p.X, (float)p.Y, (float)p.Z),
                        new Vector3((float)n.X, (float)n.Y, (float)n.Z));
                }
            }
        }

        void SmoothGroup(Stack<AVertex> avertices)
        {
            bool Merge(List<AVertex> a, List<AVertex> b)
            {
                foreach (var elemA in a)
                {
                    foreach (var elemB in b)
                    {
                        if (JVector.Dot(JVector.Normalize(elemA.Normal),
                           JVector.Normalize(elemB.Normal)) > 0.5d)
                        {
                            a.AddRange(b);
                            b.Clear();
                            return true;
                        }
                    }
                }
                return false;
            }

            var groups = new List<List<AVertex>>();

            while (avertices.Count > 0)
            {
                List<AVertex> group = new List<AVertex>();
                group.Add(avertices.Pop());
                groups.Add(group);
            }

        again:
            for (int i = 0; i < groups.Count; i++)
            {
                for (int e = 0; e < groups.Count; e++)
                {
                    if (i > e)
                    {
                        if (Merge(groups[i], groups[e]))
                            goto again;
                    }
                }
            }

            foreach (var group in groups)
            {
                JVector normal = JVector.Zero;
                foreach (var elem in group) normal += elem.Normal;
                normal.Normalize();
                foreach (var elem in group) elem.Normal = normal;
            }
        }

        public override void Build()
        {
            this.MakeHull(4);
        }

        public override void Load()
        {
            phong = new PhongShader();
            base.Load();
        }

        public override void Draw(FrameEventArgs e)
        {
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.Enable(EnableCap.Blend);

            phong.Use();
            phong.SetDefaultMaterial();

            phong.Alpha.Set(Alpha);

            phong.ViewPosition.Set(ref Showcase.Current.Camera.Position);
            phong.Projection.Set(ref Showcase.Current.Camera.ProjectionMatrix);
            phong.View.Set(ref Showcase.Current.Camera.ViewMatrix);
            phong.Model.Set(this.WorldMatrix);

            base.Draw(e);

            GL.Disable(EnableCap.Blend);
        }

        public override void Dispose()
        {
            phong.Dispose();
            base.Dispose();
        }

        public override void Update(FrameEventArgs e)
        {
            this.WorldMatrix = Matrix4.CreateRotationX(Angles.X);
            this.WorldMatrix *= Matrix4.CreateRotationY(Angles.Y);
            this.WorldMatrix *= Matrix4.CreateRotationZ(Angles.Z);
            this.WorldMatrix *= Matrix4.CreateTranslation(this.Position);
        }
    }
}