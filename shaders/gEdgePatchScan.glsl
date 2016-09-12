#version 430
#extension GL_ARB_shader_draw_parameters : require

layout(triangles) in;
layout(triangle_strip, max_vertices = 21) out;

layout(std140, binding = 1) buffer trafo {
    mat4 mMat[];
};

struct Position {
    vec3 world; // world space
    vec4 clip; // clip space
};

struct Vertex {
    vec3 trans; // translation vector v_next.pos.world - v_in.pos.world
    Position pos;
} v_next[3];

uniform mat4 uVPMat;


in VertexData {
    vec3 normal;
    vec3 pos_object; // object space
    Position pos;
    int instanceID;
} v_in[];

// emits a trinagle (v0, v1, v2)
// where vi is in clip coordinates
void emitTriangle(vec4 v0, vec4 v1, vec4 v2) {
    gl_Position = v0;
    EmitVertex();
    gl_Position = v1;
    EmitVertex();
    gl_Position = v2;
    EmitVertex();
    EndPrimitive();
}

// 2-3
// |\|
// 0-1
void emitPatch(Position v0, Position v1, Position v2, Position v3) {
    emitTriangle(v0.clip, v1.clip, v2.clip);
    emitTriangle(v2.clip, v1.clip, v3.clip);
}
// emits the patch generated from triangle edge (a,b)
// for now: a patch is only emitted if both vertices of the edge are translated in positive or negative triangle normal direction,
// 			so the case of two translated vertices on different sides of the original triangle is not handled.
// a = first vertex index of edge
// b = second vertex index of edge
// d[i] = part of the i-th vertex translation in triangle normal direction
void emitEdgePatch(float d[3], int a, int b) {
    if (d[a] > 0.0 && d[b] > 0.0) {
        emitPatch(v_in[a].pos, v_in[b].pos, v_next[a].pos, v_next[b].pos);
    } else if (d[a] < 0.0 && d[b] < 0.0) {
        emitPatch(v_next[a].pos, v_next[b].pos, v_in[a].pos, v_in[b].pos);
    }
}

void main() {
    vec3 normal = v_in[0].normal; // normal of triangle
    if (v_in[0].instanceID + 1 < mMat.length()) {
        mat4 M_next = mMat[v_in[0].instanceID + 1]; // the next translation

        // calculate properties of next triangle
        for(int i = 0; i < 3; i++) {
            v_next[i].pos.world = (M_next * vec4(v_in[i].pos_object, 1)).xyz;
            v_next[i].trans = v_next[i].pos.world - v_in[i].pos.world;
            v_next[i].pos.clip = uVPMat * vec4(v_next[i].pos.world, 1);
        }


        // project translation of vertex 0,1,2 on triangle normal
        float d[3];
        d[0] = dot(v_next[0].trans, normal);
        d[1] = dot(v_next[1].trans, normal);
        d[2] = dot(v_next[2].trans, normal);

        // emit patch for each triangle edge
        emitEdgePatch(d, 0, 1);
        emitEdgePatch(d, 1, 2);
        emitEdgePatch(d, 2, 0);
    }
    // emit input triangle
    emitTriangle(v_in[0].pos.clip, v_in[1].pos.clip, v_in[2].pos.clip);
}
