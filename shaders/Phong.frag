#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

// params
float ka = 0.5;
float kd = 1.0;
float ks = 0.8;
float ia = 0.1;
float p = 64.0;

void main() {
  // YOUR CODE HERE
  // la = ka * ia
  // ld
  vec3 kd = vec3(1.0, 1.0, 1.0);
  vec3 l =  u_light_pos - v_position.xyz;
  float r2 = length(l) * length(l);
  vec3 ld = kd * (u_light_intensity / r2) * max(dot(v_normal.xyz, normalize(l)), 0.0);
  // ls
  vec3 v = u_cam_pos - v_position.xyz;
  vec3 h = normalize(normalize(l) + normalize(v));
  vec3 ls = ks * (u_light_intensity / r2) * pow(max(dot(v_normal.xyz, h), 0.0), p);
  // color
  vec3 color = ka * ia + ld + ls;
  
  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color = vec4(color, 0.0);
  out_color.a = 1.0;
}

