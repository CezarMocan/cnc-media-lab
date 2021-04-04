#version 150

in vec2 vTexCoord;
out vec4 vFragColor;

uniform sampler2DRect tex0;
uniform float uTime;
uniform vec4 color;

float random (in vec2 st) {
    return fract(sin(dot(st.xy,
                         vec2(12.9898,78.233)))
                * 43758.5453123);
}

float noise (in vec2 st) {
    vec2 i = floor(st);
    vec2 f = fract(st);

    // Four corners in 2D of a tile
    float a = random(i);
    float b = random(i + vec2(1.0, 0.0));
    float c = random(i + vec2(0.0, 1.0));
    float d = random(i + vec2(1.0, 1.0));

    // Smooth Interpolation

    // Cubic Hermine Curve.  Same as SmoothStep()
    vec2 u = f*f*(3.0-2.0*f);
    // u = smoothstep(0.,1.,f);

    // Mix 4 coorners percentages
    return mix(a, b, u.x) +
            (c - a)* u.y * (1.0 - u.x) +
            (d - b) * u.x * u.y;
}

float dist(vec2 a, vec2 b)
{    
	return pow(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0), 0.5);
}

void main()
{
	vec4 texColor = texture(tex0, vTexCoord);

    vec2 st = vTexCoord.xy/1024.;
    st.y *= 1.;

	if (texColor.r == 0) {
        discard;
	} else {
        // Squares
        /*
        float sxPos = st.x * 100.;
        float syPos = st.y * 100.;
        if (abs(syPos) - floor(syPos) < 0.2 || abs(sxPos) - floor(sxPos) < 0.2) {
            if (abs(syPos) - floor(syPos) < 0.25 && abs(sxPos) - floor(sxPos) < 0.25)
                //vFragColor = vec4(1., 0., 0., 1.);
                vFragColor = vec4(31. / 255., 126. / 255., 240. / 255., 1.);
            else
                //vFragColor = vec4(31. / 255., 126. / 255., 240. / 255., 1.);
                discard;
        }
        else
            discard;
        */

        // Grid
        /*
        float sxPos = st.x * 100.;
        float syPos = st.y * 100.;
        if (abs(syPos) - floor(syPos) < 0.2 || abs(sxPos) - floor(sxPos) < 0.2) {    
            vFragColor = vec4(31. / 255., 126. / 255., 240. / 255., 1.);
        }
        else
            discard;
        */

        // Broken grid
        /*
        float sxPos = st.x * 128.;
        float syPos = st.y * 128.;
        float ns = noise(st * 100 + vec2(uTime / 10000.0, uTime / 10000.0));
        float ns2 = noise(st * 120 + vec2(uTime / 12000.0, uTime / 12000.0));
        if (abs(syPos - floor(syPos)) + ns * 0.08 < 0.1) vFragColor = vec4(31. / 255., 126. / 255., 240. / 255., 1.);
        else if (abs(sxPos - floor(sxPos)) + ns2 * 0.08 < 0.1) vFragColor = vec4(31. / 255., 126. / 255., 240. / 255., 1.);
        else discard;
        */

        // Horizontal lines with noise
        
        float sxPos = st.x * 128.;
        float syPos = st.y * 128.;
        float ns = noise(st * 100 + vec2(uTime / 5000.0, uTime / 5000.0));
        if (abs(syPos - (floor(syPos) + ns * 1.)) < 0.35) {
            vFragColor = vec4(249. / 255., 206. / 255., 42. / 255., 1.); //+ vec4((random(st * 100) - 0.5) * 0.5);
        } else if (abs(sxPos - (floor(sxPos) + ns * 1.)) < 0.35) {
            vFragColor = vec4(249. / 255., 206. / 255., 42. / 255., 1.);
        }
        else
            discard;
        

        // Horizontal lines
        /*
        float sxPos = st.x * 128.;
        float syPos = st.y * 128.;
        float ns = noise(st * 100 + vec2(uTime / 10000.0, uTime / 10000.0));
        if (abs(syPos - floor(syPos)) < 0.1) {
            vFragColor = vec4(31. / 255., 126. / 255., 240. / 255., 1.);
        }
        else
            discard;
        */
	}
}
