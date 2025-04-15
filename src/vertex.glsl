/*
VERTEX SHADER

Runs on each vertex of our 3D model.
It takes inputs like vertex position and normals and outputs a position in screen space, among other data.
*/

/*
uniform variables (preceded with "u"): global variables, same across all vertices
- uP: projection matrix (camera perspective; transforms 3D space into 2D perspective)
- uV: view matrix (camera position and rotation; moves camera in 3D space by transforming world in camera's view)
- uM: model matrix (object position, rotation, scale; transforms object space to world space)
- uN: normal matrix (object normals; transforms object's normal vectors correctly when object is transformed; used for lighting)
*/
uniform mat4 uP, uV, uM;
uniform mat3 uN;

/*
attribute variables (preceded with "a"): per-vertex input data
- aVertex: vertex position
- aNormal: vertex normal
- aTexCoord: vertex texture coordinates
*/
attribute vec3 aVertex, aNormal;
attribute vec2 aTexCoord;

/*
varying variables: outputted from vertex shader → passed to fragment shader
- vVertex: vertex position in model space
- vNormal: normal vector in model space
*/
varying vec3 vVertex, vNormal; 

void main() {
  // gl_Position: vertex position in screen space (special built-in output of vertex shader, used by GPU to rasterize triangles)
  gl_Position = uP * uV * uM * vec4(aVertex, 1.0);
  vVertex = aVertex;
  vNormal = aNormal;

}
