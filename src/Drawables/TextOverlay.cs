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
using System.Collections.Generic;

using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Graphics;

using OpenTK.Windowing.Common;
using System.IO;
using System.IO.Compression;

using Font = UEPADemo.Monospace12;

namespace UEPADemo
{
    public class TextOverlay : IDrawableComponent
    {
        private readonly List<float> vertices = new ();
        private readonly List<uint> indices = new ();

        private QuadShader shader;
        private VertexArrayHandle VAO;
        private BufferHandle VBO, EBO;
        private TextureHandle texture;

        private Matrix4 worldMatrix = Matrix4.Identity;

        public string Text { set; get; }

        private const int asciiMin = 32;
        private const int asciiMax = 126;
        private const int totalChars = asciiMax - asciiMin + 1;

        private TextureHandle CreateTexture(TextureUnit unit)
        {
            TextureHandle txt = GL.GenTexture();

            GL.ActiveTexture(unit);
            GL.BindTexture(TextureTarget.Texture2d, txt);

            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest);
            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest);

            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureWrapS, (int)TextureWrapMode.ClampToEdge);
            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureWrapT, (int)TextureWrapMode.ClampToEdge);

            unsafe
            {
                byte[] raw = Monospace12.GetData();
                fixed(void* ptr = &raw[0])
                {
                    GL.TexImage2D(TextureTarget.Texture2d, 0, OpenTK.Graphics.OpenGL.InternalFormat.Rgba,  Monospace12.TextureWidth, Monospace12.TextureHeight, 0,
                                OpenTK.Graphics.OpenGL.PixelFormat.Rgba, PixelType.UnsignedByte, (IntPtr)ptr);
                }
            }

            return txt;
        }

        private void AddGlyphs()
        {
            int len = totalChars;

            for (int i = 0; i < len; i++)
            {
                indices.Add((uint)(4 * i + 0)); indices.Add((uint)(4 * i + 1)); indices.Add((uint)(4 * i + 3));
                indices.Add((uint)(4 * i + 1)); indices.Add((uint)(4 * i + 2)); indices.Add((uint)(4 * i + 3));
            }

            for (int i = 0; i < len; i++)
            {
                float deltaW = (float)Monospace12.PixelWidth / (float)Monospace12.TextureWidth;
                float deltaH = (float)Monospace12.PixelHeight / (float)Monospace12.TextureHeight;
                int numch = Monospace12.NumberCharactersHorizontal;

                vertices.Add(0); vertices.Add(Monospace12.PixelHeight);
                vertices.Add(deltaW * (float)((i % numch) + 0)); vertices.Add(deltaH * (float)(i / numch + 1));

                vertices.Add(Monospace12.PixelWidth); vertices.Add(Monospace12.PixelHeight);
                vertices.Add(deltaW * (float)((i % numch) + 1)); vertices.Add(deltaH * (float)(i / numch + 1));

                vertices.Add(Monospace12.PixelWidth); vertices.Add(0);
                vertices.Add(deltaW * (float)((i % numch) + 1)); vertices.Add(deltaH * (float)(i / numch + 0));

                vertices.Add(0); vertices.Add(0);
                vertices.Add(deltaW * (float)((i % numch) + 0)); vertices.Add(deltaH * (float)(i / numch + 0));
            }
        }

        public void Load()
        {
            this.texture = CreateTexture(TextureUnit.Texture0);
            AddGlyphs();

            VAO = GL.GenVertexArray();
            GL.BindVertexArray(VAO);

            VBO = GL.GenBuffer();
            EBO = GL.GenBuffer();

            GL.BindBuffer(BufferTargetARB.ArrayBuffer, VBO);
            GL.BufferData(BufferTargetARB.ArrayBuffer, vertices.ToArray(), BufferUsageARB.StaticDraw);

            GL.BindBuffer(BufferTargetARB.ElementArrayBuffer, EBO);
            GL.BufferData(BufferTargetARB.ElementArrayBuffer, indices.ToArray(), BufferUsageARB.StaticDraw);

            GL.VertexAttribPointer(0, 2, VertexAttribPointerType.Float, false, 4 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 2, VertexAttribPointerType.Float, false, 4 * sizeof(float), 2 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            shader = new QuadShader();

            this.Text = string.Empty;
        }

        public Vector2 Position { get; set; }

        readonly string[] linebreaks = new string[] { "\r\n", "\r", "\n" };

        public void Draw(FrameEventArgs e)
        {
            GL.BindTexture(TextureTarget.ProxyTexture2d, texture);
            GL.BindVertexArray(VAO);

            shader.Use();

            float w = Showcase.Current.Window.Size.X;
            float h = Showcase.Current.Window.Size.Y;

            Matrix4 m = Matrix4.CreateOrthographicOffCenter(0.0f, w, h, 0, +1f, -1f);

            shader.Projection.Set(m);
            shader.FontTexture.Set(TextureUnit.Texture0);

            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.Enable(EnableCap.Blend);
            GL.Disable(EnableCap.DepthTest);

            string[] lines = this.Text.Split(linebreaks, StringSplitOptions.None);

            for (int i = 0; i < lines.Length; i++)
            {
                for (int k = 0; k < lines[i].Length; k++)
                {
                    int index = (int)lines[i][k];
                    int pointer = (index - asciiMin) * 6;

                    shader.Offset.Set(Position + new Vector2((Monospace12.PixelWidth - Monospace12.ExtraMargin) * k, (Monospace12.PixelHeight - Monospace12.ExtraMargin) * i));
                    GL.DrawElements(PrimitiveType.Triangles, 6, DrawElementsType.UnsignedInt, sizeof(float) * pointer);
                }
            }

            GL.Disable(EnableCap.Blend);
            GL.Enable(EnableCap.DepthTest);
        }

        public void Update(FrameEventArgs e)
        {
            // nothing to do
        }

        public void Dispose()
        {
            shader.Dispose();
            GL.DeleteTexture(texture);
            GL.DeleteVertexArray(VAO);
            GL.DeleteBuffer(VBO);
            GL.DeleteBuffer(EBO);
        }
    }

    public static class Monospace12
    {
        static readonly string atlas = @"
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
//8DAEAzsTwAAAIA";

        public static byte[] GetData()
        {
            static byte[] Decompress(byte[] inputData)
            {
                using var cs = new MemoryStream(inputData);
                using var ds = new MemoryStream();

                using (var gzs = new BufferedStream(new GZipStream(cs, CompressionMode.Decompress), 1024))
                {
                    gzs.CopyTo(ds);
                }

                return ds.ToArray();
            }

            string st = atlas.Replace("\n", "").Replace("\r", "").Replace(" ", "");
            return Decompress(System.Convert.FromBase64String(st));
        }

        public static int TextureWidth { get => 256; }
        public static int TextureHeight { get => 128; }
        public static int ExtraMargin { get => 0; }
        public static int PixelWidth { get => 10; }
        public static int PixelHeight { get => 19; }
        public static int NumberCharactersHorizontal { get => 25; }
    }

}