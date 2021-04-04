#version 150

in vec2 vTexCoord;
out vec4 vFragColor;

uniform sampler2DRect tex0;

float random (in vec2 st) {
    return fract(sin(dot(st.xy,
                         vec2(12.9898,78.233)))
                * 43758.5453123);
}

void main()
{
	vec4 texColor = texture(tex0, vTexCoord);
    vec2 st = vTexCoord.xy / 2048.0;
    vec3 noise = vec3(random(st)) * 0.05;
    float alpha = 0.;
    vFragColor = texColor + vec4(noise, alpha);
}
