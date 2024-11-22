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

namespace UEPADemo
{
    public class PhongShader : BasicShader, IModelViewProjection, IViewPosition
    {
        public UniformMatrix4 Model { private set; get; }
        public UniformMatrix4 View { private set; get; }
        public UniformMatrix4 Projection { private set; get; }
        public UniformVector3 ViewPosition { private set; get; }
        public UniformVector3 Ambient { private set; get; }
        public UniformVector3 Diffuse { private set; get; }
        public UniformVector3 Specular { private set; get; }
        public UniformFloat Shininess { private set; get; }
        public UniformFloat Alpha { private set; get; }

        public PhongShader() : base(vshader, fshader)
        {
            Model = this.Uniforms["model"] as UniformMatrix4;
            View = this.Uniforms["view"] as UniformMatrix4;
            Projection = this.Uniforms["projection"] as UniformMatrix4;
            ViewPosition = this.Uniforms["viewPos"] as UniformVector3;
            Ambient = this.Uniforms["material.ambient"] as UniformVector3;
            Diffuse = this.Uniforms["material.diffuse"] as UniformVector3;
            Specular = this.Uniforms["material.specular"] as UniformVector3;
            Shininess = this.Uniforms["material.shininess"] as UniformFloat;
            Alpha = this.Uniforms["material.alpha"] as UniformFloat;
        }

        public void SetDefaultMaterial()
        {
            Ambient.Set(0.5f,0.5f,0.5f);
            Diffuse.Set(0.6f,0.6f,0.6f);
            Specular.Set(0.3f,0.3f,0.3f);
            Shininess.Set(128.0f);
            Alpha.Set(1.0f);
        }

        private static string vshader = @"
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNorm;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        out vec3 normal;
        out vec3 pos;

        void main()
        {
            gl_Position = projection * view * model * vec4(aPos, 1.0);
            normal = mat3(transpose(inverse(model))) * aNorm;
            pos = vec3(model * vec4(aPos, 1.0));
        }
        ";

        private static string fshader = @"
        #version 330 core
        struct Material {
            vec3 ambient;
            vec3 diffuse;
            vec3 specular;
            float shininess;
            float alpha;
        };

        uniform Material material;
        uniform vec3 viewPos;
    
        in vec3 normal;
        in vec3 pos;

        out vec4 FragColor;

        void main()
        {
            vec3 lightColor = vec3(1,1,1);

            vec3 lights[4];

            // three spot-lights from above,
            // one diffusive reflection from the ground
            lights[0] = vec3(-10, 20, 0);
            lights[1] = vec3(10, 20, 0);
            lights[2] = vec3(0, 10, -10);
            lights[3] = vec3(0, -10, 0);

            vec3 nn = normalize(normal);

            // ambient
            vec3 ambient = lightColor * material.ambient;
            
            vec3 diffusive = vec3(0, 0, 0);
            vec3 specular = vec3(0, 0, 0);

            for(int i=0; i<4; i++)
            {
                // diffuse
                vec3 lightDir = normalize(lights[i] - pos);
                float diff = max(-dot(nn, lightDir), 0.0);
                diffusive += lightColor * diff * material.diffuse;
            }

            for(int i=0; i<3; i++)
            {
                // specular
                vec3 lightDir = normalize(lights[i] - pos);
                vec3 viewDir = normalize(viewPos - pos);
                vec3 reflectDir = reflect(-lightDir, nn);
                float spec = pow(max(dot(viewDir, reflectDir),0.0), material.shininess);
                specular += lightColor * spec * material.specular;
            }
            
            vec3 result = ambient + 0.25*diffusive + 0.333*specular;
            FragColor = vec4(result, material.alpha);
        }
        ";
    }
}