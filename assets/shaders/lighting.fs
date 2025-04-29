#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
// in vec4 fragColor;
in vec3 fragNormal;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Output fragment color
out vec4 finalColor;

// NOTE: Add here your custom variables

#define     MAX_LIGHTS              4
#define     LIGHT_DIRECTIONAL       0
#define     LIGHT_POINT             1

struct Light {
    int enabled;
    int type;
    vec3 position;
    vec3 target;
    vec4 color;
};

// Input lighting values
uniform Light lights[MAX_LIGHTS];
uniform vec4 ambient;
uniform vec3 viewPos;

void main()
{
    // Normalize input values
    vec3 norm = normalize(fragNormal);
    vec3 lightDir = normalize(lights[0].position - fragPosition);

    // Compute the diffuse term using Lambert's cosine law
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lights[0].color.rgb * vec3(1.0, 1.0, 1.0);

    // Combine the ambient and diffuse components
    vec3 result = ambient.rgb + diffuse;
    finalColor = vec4(result, 1.0);
}