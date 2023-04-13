#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float ka = 0.5;
float kd = 1.0;
float ks = 0.8;
float ia = 0.1;
float p = 64.0;

float h(vec2 uv) {
  // You may want to use this helper function...
  // return 0.0;
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  // tbn
  vec3 t = normalize(v_tangent.xyz);
  vec3 n = normalize(v_normal.xyz);
  vec3 b = cross(t, n);
  mat3 tbn = mat3(t, b, n);
  
  float width = u_texture_2_size[0];
  float height = u_texture_2_size[1];

  float du = (h(v_uv + vec2(1.0 / width, 0.0)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dv = (h(v_uv + vec2(0.0, 1.0 / height)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  
  vec3 no = vec3(-du, -dv, 1.0);
  vec3 nd = tbn * normalize(no);
  
  // phong
  // ld
  vec3 kd = vec3(1.0, 1.0, 1.0);
  vec3 l =  u_light_pos - v_position.xyz;
  float r2 = length(l) * length(l);
  vec3 ld = kd * (u_light_intensity / r2) * max(dot(nd, normalize(l)), 0.0);
  // ls
  vec3 v = u_cam_pos - v_position.xyz;
  vec3 hhh = normalize(normalize(l) + normalize(v));
  vec3 ls = ks * (u_light_intensity / r2) * pow(max(dot(nd, hhh), 0.0), p);
  // color
  vec3 color = ka * ia + ld + ls;
  
  // (Placeholder code. You will want to replace it.)
  out_color = vec4(color, 0.0);
  out_color.a = 1.0;
}

