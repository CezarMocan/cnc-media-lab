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
        float sxPos = st.x * 100.;
        float syPos = st.y * 100.;
        if (abs(syPos) - floor(syPos) < 0.2 || abs(sxPos) - floor(sxPos) < 0.2) {
            if (abs(syPos) - floor(syPos) < 0.25 && abs(sxPos) - floor(sxPos) < 0.25)
                vFragColor = color;
            else
                discard;
        }
        else
            discard;
	}
}
