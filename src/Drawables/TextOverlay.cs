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
using System.Drawing;
using System.Collections.Generic;

using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Graphics;

using Drawing = System.Drawing;
using OpenTK.Windowing.Common;

namespace GJKEPADemo
{
    public class TextOverlay : IDrawableComponent
    {
        private List<float> vertices = new List<float>();
        private List<uint> indices = new List<uint>();

        private QuadShader shader;
        private VertexArrayHandle VAO;
        private BufferHandle VBO, EBO;
        private TextureHandle texture;

        private Matrix4 worldMatrix = Matrix4.Identity;

        public string Text { set; get; }

        public const int FontSize = 10;

        private float pxWidth = 0, pxHeight = 0;
        private int numCharsH = 0, numCharsV = 0;

        private const float extraMargin = 10.0f;
        private const float spacing = 1.2f;

        private const int asciiMin = 33;
        private const int totalChars = 256 - asciiMin;

        int textureSize = 256;

        private TextureHandle CreateTexture(TextureUnit unit)
        {
            Drawing.Graphics graphics;
            Drawing.Bitmap bitmap;
            Drawing.Font font;

            while (true)
            {
                bitmap = new Drawing.Bitmap(textureSize, textureSize);
                graphics = Graphics.FromImage(bitmap);

                graphics.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;
                graphics.PageUnit = GraphicsUnit.Pixel;

                font = new Font(FontFamily.GenericMonospace, FontSize);
                var fsize = graphics.MeasureString("#", font, new System.Drawing.PointF(), StringFormat.GenericTypographic);

                pxWidth = (int)(fsize.Width * spacing + extraMargin);
                pxHeight = (int)(fsize.Height * spacing + extraMargin);

                numCharsH = (int)((float)textureSize / pxWidth);
                numCharsV = (int)((float)textureSize / pxHeight);

                if ((numCharsH * numCharsV) >= totalChars) { break; }

                graphics.Dispose();
                textureSize *= 2;
            }

            for (int ic = asciiMin; ic < 256; ic++)
            {
                graphics.DrawString(((char)ic).ToString(), font, Brushes.Black,
                new Drawing.PointF((ic - asciiMin) % numCharsH * pxWidth, (ic - asciiMin) / numCharsH * pxHeight), StringFormat.GenericTypographic);
            }

            var data = bitmap.LockBits(new Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height),
                Drawing.Imaging.ImageLockMode.ReadOnly,
                Drawing.Imaging.PixelFormat.Format32bppArgb);

            TextureHandle txt = GL.GenTexture();

            GL.ActiveTexture(unit);
            GL.BindTexture(TextureTarget.Texture2d, txt);

            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest);
            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest);

            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureWrapS, (int)TextureWrapMode.ClampToEdge);
            GL.TexParameteri(TextureTarget.Texture2d, TextureParameterName.TextureWrapT, (int)TextureWrapMode.ClampToEdge);

            GL.TexImage2D(TextureTarget.Texture2d, 0, OpenTK.Graphics.OpenGL.InternalFormat.Rgba, data.Width, data.Height, 0,
                          OpenTK.Graphics.OpenGL.PixelFormat.Rgba, PixelType.UnsignedByte, data.Scan0);

            bitmap.UnlockBits(data);
            bitmap.Dispose();

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
                float deltaW = this.pxWidth / (float)this.textureSize;
                float deltaH = this.pxHeight / (float)this.textureSize;

                vertices.Add(0); vertices.Add(pxHeight); vertices.Add(0);
                vertices.Add(deltaW * (float)((i % this.numCharsH) + 0)); vertices.Add(deltaH * (float)(i / this.numCharsH + 1));

                vertices.Add(pxWidth); vertices.Add(pxHeight); vertices.Add(0);
                vertices.Add(deltaW * (float)((i % this.numCharsH) + 1)); vertices.Add(deltaH * (float)(i / this.numCharsH + 1));

                vertices.Add(pxWidth); vertices.Add(0); vertices.Add(0);
                vertices.Add(deltaW * (float)((i % this.numCharsH) + 1)); vertices.Add(deltaH * (float)(i / this.numCharsH + 0));

                vertices.Add(0); vertices.Add(0); vertices.Add(0);
                vertices.Add(deltaW * (float)((i % this.numCharsH) + 0)); vertices.Add(deltaH * (float)(i / this.numCharsH + 0));
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

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 5 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 2, VertexAttribPointerType.Float, false, 5 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            shader = new QuadShader();

            this.Text = string.Empty;
        }

        public Vector2 Position { get; set; }
        string[] linebreaks = new string[] { "\r\n", "\r", "\n" };

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

            float charw = pxWidth;
            float charh = pxHeight;

            string[] lines = this.Text.Split(linebreaks, StringSplitOptions.None);

            for (int i = 0; i < lines.Length; i++)
            {
                for (int k = 0; k < lines[i].Length; k++)
                {
                    int index = (int)lines[i][k];
                    int pointer = (index - asciiMin) * 6;

                    shader.Offset.Set(Position + new Vector2((charw - extraMargin) * k, (charh - extraMargin) * i));
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
    }
}