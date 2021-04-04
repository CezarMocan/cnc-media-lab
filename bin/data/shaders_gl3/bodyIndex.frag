#version 150

in float vKeep;
in vec2 vTexCoord;

out vec4 vFragColor;
uniform sampler2DRect uBodyIndexTex;
uniform float uBodyIndexToExtract;

bool floatEquals(float a, float b) {
	const float margin = 0.001;
	if (abs(a - b) < margin) return true;
	return false;
}

void main()
{
	vec4 texColor = texture(uBodyIndexTex, vTexCoord);
	if (!floatEquals(texColor.r * 255, uBodyIndexToExtract)) {
		discard;
	} else {
		float index = uBodyIndexToExtract;
		vFragColor = vec4(0.2 * index, 0.1 * index, 1.0 / index, 1.0);
	}
}
