#version 150

in vec2 vTexCoord;
out vec4 vFragColor;

uniform sampler2DRect tex0;
uniform float uTime;
uniform vec4 color;

void main()
{
	vec4 texColor = texture(tex0, vTexCoord);

    vec2 st = vTexCoord.xy/1024.;
    st.y *= 1.;

	if (texColor.r == 0) {
        discard;
	} else {
        // Vertical lines
        float sxPos = st.x * 128.;
        float syPos = st.y * 128.;
        if (abs(sxPos - floor(sxPos)) < 0.1) {
            vFragColor = color;
        }
        else
            discard;
	}
}
