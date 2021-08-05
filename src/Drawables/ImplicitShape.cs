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

//#define SMOOTHGROUPS

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
            public JVector n1;
            public JVector n2;
            public JVector n3;
            public int generation;
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

        List<AVertex> avertices = new List<AVertex>();
        List<ATriangle> atriangles = new List<ATriangle>();

        private void MakeHull(int generationThreshold)
        {
            // Based on the acticle
            // [Collision Detection Using Minkowski Difference, 2008]
            // by Ben Kenwright.
            // https://xbdev.net/physics/MinkowskiDifference/index.php

            float distanceThreshold = 0.0f;
            int counter = 0;

            if (generationThreshold < 0) generationThreshold = 4;

            Stack<ClipTriangle> activeTriList = new Stack<ClipTriangle>();

            JVector[] v = new JVector[] // 6 Array
		    {
            new JVector( -1,  0,  0 ),
            new JVector(  1,  0,  0 ),

            new JVector(  0, -1,  0 ),
            new JVector(  0,  1,  0 ),

            new JVector(  0,  0, -1 ),
            new JVector(  0,  0,  1 ),
            };

            int[,] kTriangleVerts = new int[8, 3] // 8 x 3 Array
		    {
            { 5, 1, 3 },
            { 4, 3, 1 },
            { 3, 4, 0 },
            { 0, 5, 3 },

            { 5, 2, 1 },
            { 4, 1, 2 },
            { 2, 0, 4 },
            { 0, 2, 5 }
            };

            for (int i = 0; i < 8; i++)
            {
                ClipTriangle tri = new ClipTriangle();
                tri.n1 = v[kTriangleVerts[i, 0]];
                tri.n2 = v[kTriangleVerts[i, 1]];
                tri.n3 = v[kTriangleVerts[i, 2]];
                tri.generation = 0;
                activeTriList.Push(tri);
            }

            List<JVector> pointSet = new List<JVector>();

            while (activeTriList.Count > 0)
            {
                ClipTriangle tri = activeTriList.Pop();

                JVector p1; SupportMap.SupportMapping(ref tri.n1, out p1);
                JVector p2; SupportMap.SupportMapping(ref tri.n2, out p2);
                JVector p3; SupportMap.SupportMapping(ref tri.n3, out p3);

                double d1 = (p2 - p1).LengthSquared();
                double d2 = (p3 - p2).LengthSquared();
                double d3 = (p1 - p3).LengthSquared();

                if (Math.Max(Math.Max(d1, d2), d3) > distanceThreshold && tri.generation < generationThreshold)
                {
                    ClipTriangle tri1 = new ClipTriangle();
                    ClipTriangle tri2 = new ClipTriangle();
                    ClipTriangle tri3 = new ClipTriangle();
                    ClipTriangle tri4 = new ClipTriangle();

                    tri1.generation = tri.generation + 1;
                    tri2.generation = tri.generation + 1;
                    tri3.generation = tri.generation + 1;
                    tri4.generation = tri.generation + 1;

                    tri1.n1 = tri.n1;
                    tri2.n2 = tri.n2;
                    tri3.n3 = tri.n3;

                    JVector n = 0.5d * (tri.n1 + tri.n2);
                    n.Normalize();

                    tri1.n2 = n;
                    tri2.n1 = n;
                    tri4.n3 = n;

                    n = 0.5d * (tri.n2 + tri.n3);
                    n.Normalize();

                    tri2.n3 = n;
                    tri3.n2 = n;
                    tri4.n1 = n;

                    n = 0.5d * (tri.n3 + tri.n1);
                    n.Normalize();

                    tri1.n3 = n;
                    tri3.n1 = n;
                    tri4.n2 = n;

                    activeTriList.Push(tri1);
                    activeTriList.Push(tri2);
                    activeTriList.Push(tri3);
                    activeTriList.Push(tri4);
                }
                else
                {
                    JVector n = (p3 - p1) % (p2 - p1);

                    if (n.LengthSquared() > 1e-12)
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
            var groups = avertices.GroupBy(s => s.Position);
            foreach(var group in groups) 
            {
                SmoothGroup(new Stack<AVertex>(group));
            }
#endif
   
            counter = 0;

            foreach(ATriangle atri in atriangles)
            {
                
                for(int i = 0;i < 3;i++)
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
                foreach(var elemA in a)
                {
                    foreach(var elemB in b)
                    {
                        if(JVector.Dot(JVector.Normalize(elemA.Normal), 
                           JVector.Normalize(elemB.Normal)) > 0.1d)
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

            while(avertices.Count > 0)
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
                    if (i != e)
                    {
                        if (Merge(groups[i], groups[e]))
                            goto again;
                    }
                }
            }

            foreach(var group in groups)
            {
                JVector normal = JVector.Zero;
                foreach(var elem in group) normal += elem.Normal;
                normal.Normalize();
                foreach(var elem in group) elem.Normal = normal;
            }
        }

        public override void Build()
        {
            this.MakeHull(5);
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