#version 410 core

layout(location = 0) in vec4 vPosition;
layout(location = 2) in vec3 vNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform int gizmoAxis;
uniform int hoveredAxis;
uniform float gizmoScale;
uniform mat4 gizmoTransform;

out vec3 normal;
out vec3 gizmoColor;

mat4 getGizmoAxisRot() {
    if (gizmoAxis == 0) {
        return mat4(
            0, 1, 0, 0,
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        );
    }
    if (gizmoAxis == 2) {
        return mat4(
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1
        );
    }
    return mat4(1.0);
}

vec3 getGizmoColor() {
    if (gizmoAxis == hoveredAxis) return vec3(1, 1, 0);
    if (gizmoAxis == 0) return vec3(1, 0, 0);
    if (gizmoAxis == 1) return vec3(0, 1, 0);
    if (gizmoAxis == 2) return vec3(0, 0, 1);
    return vec3(0.8);
}

void main() {
    mat4 view_ = view;
    view_[3] = vec4(0, 0, 0, 1);
    
    vec4 offset = vec4(-5.1, -4.6, 0, 1);
    vec4 pos = gizmoTransform * vPosition;
    float tmp = pos.y;
    pos.y = pos.z;
    pos.z = tmp;
    gl_Position = projection * model * view_ * pos + offset;
    
    normal = normalize(vNormal);
    gizmoColor = getGizmoColor();
}