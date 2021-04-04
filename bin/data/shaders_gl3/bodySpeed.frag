#version 150

in vec2 vTexCoord;

out vec4 vFragColor;

uniform sampler2DRect tex0;
uniform vec3 vLeftWrist;
uniform vec3 vRightWrist;
uniform vec3 vLeftShoulder;
uniform vec3 vRightShoulder;
uniform vec3 vLeftHip;
uniform vec3 vRightHip;
uniform vec3 vHead;
uniform float uTime;
uniform float uArmsParam;
uniform float uHueShiftParam;
//uniform float uBodyIndex;


float random (in vec2 st) {
    return fract(sin(dot(st.xy,
                         vec2(12.9898,78.233)))
                * 43758.5453123);
}

// 2D Noise based on Morgan McGuire @morgan3d
// https://www.shadertoy.com/view/4dS3Wd
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

mat2 rotate2d(float angle){
    return mat2(cos(angle),-sin(angle),
                sin(angle),cos(angle));
}

float lines(in vec2 pos, float b){
    float scale = 10.0;
    pos *= scale;
    return smoothstep(0.0,
                    .5+b*.5,
                    abs((sin(pos.x*3.1415)+b*2.0))*.5);
}

// All components are in [0..1]
vec3 rgb2hsv(vec3 c)
{
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

// All components are in [0..1]
vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

float dist(vec2 a, vec2 b)
{    
	return pow(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0), 0.5);
}

float colorFn(vec2 texCoord, vec3 joint) 
{
	return (1 - (dist(texCoord.xy, joint.xy) * (random(texCoord))) / joint.z);
}

void main()
{
	vec4 texColor = texture(tex0, vTexCoord);

    vec2 st = vTexCoord.xy/1024.;
    st.y *= 1.;

    vec2 pos = st.yx*vec2(3.,2.);

    float pattern = pos.x;

    // Add noise
    // pos = rotate2d( noise(pos + (uTime / 100.0) / tan(uTime / 60.0)) / 1.0 ) * pos;
    /// pos = rotate2d(noise(pos) + (uTime / 10000.0)) * pos;

    pos = rotate2d(noise(2. * pos) + (uTime / 10000.0) + 0.8) * pos;

    // Draw lines
    pattern = lines(pos,.75);
    float pattern2 = lines(pos + 0.01,.75);
    float pattern3 = lines(pos + 0.025, .75);


    //gl_FragColor = vec4(pattern, pattern2, pattern3, 1.0);
	

	if (texColor.r == 0) {
        discard;
	} else {
		//float index = ((texColor.r / 20.0) * 255) + 1;
        /*
		float r = 0.0;
		r = max(r, colorFn(vTexCoord, vLeftWrist));
		r = max(r, colorFn(vTexCoord, vRightWrist));
		r = max(r, colorFn(vTexCoord, vLeftShoulder));
		r = max(r, colorFn(vTexCoord, vRightShoulder));
		r = max(r, colorFn(vTexCoord, vLeftHip));
		r = max(r, colorFn(vTexCoord, vRightHip));
		r = max(r, colorFn(vTexCoord, vHead));
		r = r * (abs(sin(uTime / 2000.0 + vTexCoord.x + vTexCoord.y)) / 5 + 0.8);
		
		// vec4 speedColor = vec4((1.2 * r), vTexCoord.y / 1024.0 * 0.5, (sin(uTime / 1000.0) + 1.0) / 2.0, 1.0);
        vec4 speedColor = vec4(vTexCoord.y / 1024.0 * 0.5, (1.2 * r), 1.0 - uArmsParam, 1.0);
        
        vec4 patternColor = vec4(pattern, pattern2, pattern3, 1.0);
        vec4 finalColor = mix(speedColor, patternColor, r);

        vec3 hsvFinalColor = rgb2hsv(finalColor.rgb);
        hsvFinalColor.r = mod(hsvFinalColor.r + uHueShiftParam + sin(uTime / 5000) / 2.0, 1.0);
        if (hsvFinalColor.r < 0.) hsvFinalColor.r = 0.;
        if (hsvFinalColor.r > 1.) hsvFinalColor.r = 1.;
        vFragColor = vec4(hsv2rgb(hsvFinalColor), finalColor.a);
        */

        /*
        float ns = dist(st, vec2(0, 1));
        vec3 color1 = vec3(1.0,0.55,0);
        vec3 color2 = vec3(0.226,0.000,0.615);
        vec3 color = mix(color1, color2, ns);

        float rnd = (random(st.yx*vec2(3.,2.) * 10.) * 0.1 - 0.05);
        vFragColor = vec4(color.r + rnd, color.g + rnd, color.b + rnd, 1.0);
        */

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
