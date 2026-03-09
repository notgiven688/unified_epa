// UnifiedEPA
// (c) Thorben Linneweber
// SPDX-License-Identifier: MIT
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices.JavaScript;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace UEPADemo
{
    public class Playground
    {
        private Camera3D camera;
        private Shader shader;
        private Font font;

        private readonly ISupportMappable[] allShapes = new ISupportMappable[]
        {
            new CylinderShape(),
            new CubeShape(),
            new LineShape(),
            new TriangleShape(),
            new SphereShape(),
            new EllipsoidShape(),
            new CappedConeShape(),
            new CapsuleShape(),
            new ConeShape(),
        };
        private readonly Random rng = new();

        private ISupportMappable shapeLeft;
        private ISupportMappable shapeRight;

        private float distance = 1.7f;
        private float angleTime = 0f;
        private bool autorotate = true;

        private Mesh meshLeft;
        private Mesh meshRight;
        private Model modelLeft;
        private Model modelRight;


        public Playground()
        {
            const int screenWidth = 800;
            const int screenHeight = 600;

            SetConfigFlags(ConfigFlags.Msaa4xHint);
            InitWindow(screenWidth, screenHeight, "UEPA WebDemo");

            camera = new()
            {
                Position = new Vector3(0.0f, 0.0f, -6.0f),
                Target = new Vector3(0.0f, 0.0f, 0.0f),
                Up = new Vector3(0.0f, 1.0f, 0.0f),
                FovY = 45.0f,
                Projection = CameraProjection.Perspective
            };

            shader = LoadShader("assets/lighting.vs", "assets/lighting.fs");

            unsafe
            {
                shader.Locs[(int)ShaderLocationIndex.MatrixModel] = GetShaderLocation(shader, "matModel");
                shader.Locs[(int)ShaderLocationIndex.VectorView] = GetShaderLocation(shader, "viewPos");
            }

            int ambientLoc = GetShaderLocation(shader, "ambient");
            Raylib.SetShaderValue(shader, ambientLoc,
                new float[] { 0.3f, 0.3f, 0.3f, 1.0f }, ShaderUniformDataType.Vec4);

            Rlights.CreateLight(0, LightType.Point, new Vector3(-10, 20, 0), Vector3.Zero, Color.White, shader);
            Rlights.CreateLight(1, LightType.Point, new Vector3(10, 20, 0), Vector3.Zero, Color.White, shader);
            Rlights.CreateLight(2, LightType.Point, new Vector3(0, 10, -10), Vector3.Zero, Color.White, shader);

            font = LoadFontEx("assets/JetBrainsMono-Regular.ttf", 20, null, 0);

            SetTargetFPS(60);

            shapeLeft = new CubeShape();
            shapeRight = new SphereShape();

            BuildShapeMesh(shapeLeft, out meshLeft, out modelLeft);
            BuildShapeMesh(shapeRight, out meshRight, out modelRight);
        }

        public void Close()
        {
            UnloadModel(modelLeft);
            UnloadModel(modelRight);
            UnloadFont(font);
            UnloadShader(shader);
            CloseWindow();
        }

        private void NextRandomShapes()
        {
            UnloadModel(modelLeft);
            UnloadModel(modelRight);

            int l = rng.Next(allShapes.Length);
            int r = rng.Next(allShapes.Length);

            shapeLeft = allShapes[l];
            shapeRight = allShapes[r];

            BuildShapeMesh(shapeLeft, out meshLeft, out modelLeft);
            BuildShapeMesh(shapeRight, out meshRight, out modelRight);
        }

        private void BuildShapeMesh(ISupportMappable supportMap, out Mesh mesh, out Model model)
        {
            var vertices = new List<float>();
            var normals = new List<float>();

            MakeHull(supportMap, 4, vertices, normals);

            mesh = new Mesh();
            int vertexCount = vertices.Count / 3;
            mesh.VertexCount = vertexCount;
            mesh.TriangleCount = vertexCount / 3;

            unsafe
            {
                float* verts = (float*)Raylib_cs.Raylib.MemAlloc((uint)(vertices.Count * sizeof(float)));
                float* norms = (float*)Raylib_cs.Raylib.MemAlloc((uint)(normals.Count * sizeof(float)));

                for (int i = 0; i < vertices.Count; i++) verts[i] = vertices[i];
                for (int i = 0; i < normals.Count; i++) norms[i] = normals[i];

                mesh.Vertices = verts;
                mesh.Normals = norms;
            }

            UploadMesh(ref mesh, false);

            model = LoadModelFromMesh(mesh);
            unsafe
            {
                model.Materials[0].Shader = shader;
            }
        }

        private static void MakeHull(ISupportMappable supportMap, int subdivisions,
            List<float> outVertices, List<float> outNormals)
        {
            double gr = (1.0d + Math.Sqrt(5.0d)) / 2.0d;

            JVector[] icoVerts = new JVector[12]
            {
                new(0, +1, +gr), new(0, -1, +gr), new(0, +1, -gr), new(0, -1, -gr),
                new(+1, +gr, 0), new(+1, -gr, 0), new(-1, +gr, 0), new(-1, -gr, 0),
                new(+gr, 0, +1), new(+gr, 0, -1), new(-gr, 0, +1), new(-gr, 0, -1),
            };

            int[,] indices = new int[20, 3]
            {
                { 1, 0, 10 }, { 0, 1, 8 }, { 0, 4, 6 }, { 4, 0, 8 }, { 0, 6, 10 },
                { 5, 1, 7 }, { 1, 5, 8 }, { 7, 1, 10 },
                { 2, 3, 11 }, { 3, 2, 9 }, { 4, 2, 6 }, { 2, 4, 9 }, { 6, 2, 11 },
                { 3, 5, 7 }, { 5, 3, 9 }, { 3, 7, 11 },
                { 4, 8, 9 }, { 8, 5, 9 }, { 10, 6, 11 }, { 7, 10, 11 }
            };

            var stack = new Stack<(JVector v1, JVector v2, JVector v3, int div)>();

            for (int i = 0; i < 20; i++)
                stack.Push((icoVerts[indices[i, 0]], icoVerts[indices[i, 1]], icoVerts[indices[i, 2]], 0));

            var triPositions = new List<JVector>();
            var triNormals = new List<JVector>();

            while (stack.Count > 0)
            {
                var (v1, v2, v3, div) = stack.Pop();

                if (div < subdivisions)
                {
                    JVector m12 = (v1 + v2) / 2.0d;
                    JVector m23 = (v2 + v3) / 2.0d;
                    JVector m31 = (v3 + v1) / 2.0d;

                    stack.Push((v1, m12, m31, div + 1));
                    stack.Push((m12, v2, m23, div + 1));
                    stack.Push((m31, m23, v3, div + 1));
                    stack.Push((m12, m23, m31, div + 1));
                }
                else
                {
                    supportMap.SupportMapping(v1, out JVector p1);
                    supportMap.SupportMapping(v2, out JVector p2);
                    supportMap.SupportMapping(v3, out JVector p3);
                    JVector n = (p2 - p1) % (p3 - p1);

                    if (n.LengthSquared() > 1e-24d)
                    {
                        triPositions.Add(p1); triPositions.Add(p2); triPositions.Add(p3);
                        triNormals.Add(n); triNormals.Add(n); triNormals.Add(n);
                    }
                }
            }

            // Smooth normals: group vertices by position and average normals within smooth groups
            var vertexGroups = new Dictionary<(long, long, long), List<int>>();
            for (int i = 0; i < triPositions.Count; i++)
            {
                var p = triPositions[i];
                var key = (BitConverter.DoubleToInt64Bits(p.X),
                           BitConverter.DoubleToInt64Bits(p.Y),
                           BitConverter.DoubleToInt64Bits(p.Z));
                if (!vertexGroups.ContainsKey(key))
                    vertexGroups[key] = new List<int>();
                vertexGroups[key].Add(i);
            }

            var smoothedNormals = new JVector[triNormals.Count];
            for (int i = 0; i < triNormals.Count; i++)
                smoothedNormals[i] = triNormals[i];

            foreach (var group in vertexGroups.Values)
            {
                // Build smooth groups with iterative merging (transitive closure)
                var smoothGroups = new List<List<int>>();
                foreach (int idx in group)
                    smoothGroups.Add(new List<int> { idx });

                bool merged;
                do
                {
                    merged = false;
                    for (int i = 0; i < smoothGroups.Count && !merged; i++)
                    {
                        for (int e = 0; e < i && !merged; e++)
                        {
                            foreach (int idxA in smoothGroups[i])
                            {
                                foreach (int idxB in smoothGroups[e])
                                {
                                    if (JVector.Dot(JVector.Normalize(triNormals[idxA]),
                                        JVector.Normalize(triNormals[idxB])) > 0.5d)
                                    {
                                        smoothGroups[i].AddRange(smoothGroups[e]);
                                        smoothGroups[e].Clear();
                                        merged = true;
                                        break;
                                    }
                                }
                                if (merged) break;
                            }
                        }
                    }
                } while (merged);

                foreach (var sg in smoothGroups)
                {
                    JVector avg = JVector.Zero;
                    foreach (int idx in sg) avg += triNormals[idx];
                    avg.Normalize();
                    foreach (int idx in sg) smoothedNormals[idx] = avg;
                }
            }

            for (int i = 0; i < triPositions.Count; i++)
            {
                outVertices.Add((float)triPositions[i].X);
                outVertices.Add((float)triPositions[i].Y);
                outVertices.Add((float)triPositions[i].Z);

                outNormals.Add((float)smoothedNormals[i].X);
                outNormals.Add((float)smoothedNormals[i].Y);
                outNormals.Add((float)smoothedNormals[i].Z);
            }
        }

        private Matrix4x4 BuildTransform(Vector3 angles, Vector3 position)
        {
            return Matrix4x4.CreateRotationX(angles.X)
                 * Matrix4x4.CreateRotationY(angles.Y)
                 * Matrix4x4.CreateRotationZ(angles.Z)
                 * Matrix4x4.CreateTranslation(position);
        }

        private JMatrix ToJMatrix(Matrix4x4 m)
        {
            JMatrix result;
            result.M11 = m.M11; result.M12 = m.M12; result.M13 = m.M13;
            result.M21 = m.M21; result.M22 = m.M22; result.M23 = m.M23;
            result.M31 = m.M31; result.M32 = m.M32; result.M33 = m.M33;
            return result;
        }

        private Vector3 anglesLeft = Vector3.Zero;
        private Vector3 anglesRight = Vector3.Zero;

        public void UpdateFrame()
        {
            float dt = GetFrameTime();

            if (IsKeyPressed(KeyboardKey.G) || IsMouseButtonPressed(MouseButton.Left))
            {
                NextRandomShapes();
            }

            if (IsKeyPressed(KeyboardKey.R))
            {
                autorotate = !autorotate;
            }

            if (IsKeyDown(KeyboardKey.K)) distance -= 2.0f * dt;
            if (IsKeyDown(KeyboardKey.L)) distance += 2.0f * dt;

            float wheel = GetMouseWheelMove();
            if (wheel != 0.0f)
            {
                Vector3 dir = Vector3.Normalize(camera.Target - camera.Position);
                Vector3 newPos = camera.Position + dir * wheel * 0.5f;
                if (Vector3.Distance(newPos, camera.Target) > 2.0f)
                    camera.Position = newPos;
            }

            if (autorotate)
            {
                angleTime += dt;
                Vector3 deltaAngle = new Vector3(0.36f, 0.42f, 0.48f) * dt;
                anglesLeft += deltaAngle;
                anglesRight -= deltaAngle;
            }

            Vector3 posLeft = new Vector3(+distance / 2.0f, 0.0f, 0.0f);
            Vector3 posRight = new Vector3(-distance / 2.0f, 0.3f, 0.1f);

            Matrix4x4 transformLeft = BuildTransform(anglesLeft, posLeft);
            Matrix4x4 transformRight = BuildTransform(anglesRight, posRight);

            // Run EPA detection
            JMatrix orientLeft = ToJMatrix(transformLeft);
            JMatrix orientRight = ToJMatrix(transformRight);
            JVector jPosLeft = new JVector(posLeft.X, posLeft.Y, posLeft.Z);
            JVector jPosRight = new JVector(posRight.X, posRight.Y, posRight.Z);

            bool success = UEPA.Detect(shapeLeft, shapeRight,
                orientLeft, orientRight, jPosLeft, jPosRight,
                out JVector pl, out JVector pr, out double separation);

            bool colliding = success && separation < 0.0d;
            float alpha = colliding ? 0.5f : 1.0f;

            unsafe
            {
                Raylib.SetShaderValue(shader,
                    shader.Locs[(int)ShaderLocationIndex.VectorView],
                    camera.Position, ShaderUniformDataType.Vec3);
            }

            BeginDrawing();
            ClearBackground(new Color(63, 66, 73, 255));

            BeginMode3D(camera);

            // Draw shapes
            Color shapeColor = new Color((byte)140, (byte)160, (byte)200, (byte)(alpha * 255));

            // Transpose: System.Numerics row-major -> Raylib column-major
            modelLeft.Transform = Matrix4x4.Transpose(transformLeft);
            modelRight.Transform = Matrix4x4.Transpose(transformRight);

            if (colliding) BeginBlendMode(BlendMode.Alpha);

            DrawModel(modelLeft, Vector3.Zero, 1.0f, shapeColor);
            DrawModel(modelRight, Vector3.Zero, 1.0f, shapeColor);

            if (colliding) EndBlendMode();

            // Draw connection line and points on top
            if (success)
            {
                Vector3 pA = new Vector3((float)pl.X, (float)pl.Y, (float)pl.Z);
                Vector3 pB = new Vector3((float)pr.X, (float)pr.Y, (float)pr.Z);

                Color neonGreen = new Color((byte)57, (byte)255, (byte)20, (byte)255);
                Rlgl.DrawRenderBatchActive();
                Rlgl.DisableDepthTest();
                DrawLine3D(pA, pB, neonGreen);
                DrawSphere(pA, 0.04f, neonGreen);
                DrawSphere(pB, 0.04f, neonGreen);
                Rlgl.DrawRenderBatchActive();
                Rlgl.EnableDepthTest();
            }

            EndMode3D();

            // HUD text
            const int fontSize = 20;
            const int spacing = 1;
            string leftName = shapeLeft.GetType().Name.Replace("Shape", "");
            string rightName = shapeRight.GetType().Name.Replace("Shape", "");

            string fpsText = $"{GetFPS()} fps";
            DrawTextEx(font, fpsText, new Vector2(10, 10), fontSize, spacing, Color.RayWhite);
            DrawTextEx(font, fpsText, new Vector2(11, 10), fontSize, spacing, Color.RayWhite);
            DrawTextEx(font, $"UEPA ({leftName} / {rightName})", new Vector2(10, 35), fontSize, spacing, Color.RayWhite);

            if (success)
            {
                DrawTextEx(font, $"Collision: {colliding}", new Vector2(10, 60), fontSize, spacing, Color.RayWhite);
                DrawTextEx(font, $"Separation: {separation:0.###}", new Vector2(10, 85), fontSize, spacing, Color.RayWhite);
                DrawTextEx(font, $"Iterations: {UEPA.epaSolver.Statistics.Iterations}", new Vector2(10, 110), fontSize, spacing, Color.RayWhite);
                DrawTextEx(font, $"Accuracy: {UEPA.epaSolver.Statistics.Accuracy:0E+00}", new Vector2(10, 135), fontSize, spacing, Color.RayWhite);
            }

            EndDrawing();
        }
    }

    public partial class Application
    {
        private static Playground _playground;

        [JSExport]
        public static void UpdateFrame()
        {
            _playground?.UpdateFrame();
        }

        [JSExport]
        public static void Resize(int width, int height)
        {
            SetWindowSize(width, height);
        }

        public static void Main()
        {
            _playground = new Playground();
        }
    }
}
