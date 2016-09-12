#version 430
#extension GL_ARB_shader_draw_parameters : require
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

layout(std140, binding = 1) buffer trafo {
    mat4 mMat[];
};

layout(std140, binding = 2) buffer trafo_normal {
    mat3 nMat[];
};

struct Position {
    vec3 world; // world space
    vec4 clip; // clip space
};

in vec3 a_position;
in vec3 a_normal;

uniform mat4 uVPMat;

out VertexData {
    vec3 normal;
    vec3 pos_object; // object space
    Position pos;
    int instanceID;
} v_out;


void main() {
    v_out.instanceID = gl_InstanceID + gl_BaseInstanceARB;
    vec4 p = mMat[v_out.instanceID] * vec4(a_position,1.0);
    v_out.pos_object = a_position;
    v_out.normal = normalize(nMat[gl_InstanceID + gl_BaseInstanceARB] * a_normal);
    v_out.pos.world = p.xyz;
    v_out.pos.clip = uVPMat * p;
    gl_Position = v_out.pos.clip;
}
