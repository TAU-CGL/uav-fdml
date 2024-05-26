Scene = {}
Scene.Shaders = {
    {
        Name = "S_custom_gizmo",
        VertexSource = "/fdml/shaders/custom_gizmo.vs",
        FragmentSource = "/fdml/shaders/custom_gizmo.fs"
    }
}
Scene.Textures = {
    {
        Name = "T_room",
        Path = "/fdml/scans/240521-141038/240521-141038.jpg"
    }
}
Scene.Materials = {
    {
        Name = "M_room",
        ShaderName = "S_default",
        DiffuseTexture = "T_room",
        Shininess = 128
    },
    {
        Name = "M_cursor",
        ShaderName = "S_default",
        DiffuseColor = {33/255, 114/255, 41/255, 0.8},
    },
    {
        Name = "M_custom_gizmo",
        ShaderName = "S_custom_gizmo",
    }

}
Scene.StaticMeshes = {
    {
        Name = "SM_room",
        Path = "/fdml/scans/240521-141038/240521-141038-scaled.obj",
        KeepData = true
    },
    {
        Name = "SM_cursor",
        Path = "/fdml/cursor.obj",
        KeepData = true
    }
}
Scene.SkeletalMeshes = {
}

Scene.Objects = {
    {
        Type = "AmbientLight",
        Name = "ambientLight",
        Intensity = 0.8,
        Color = {1, 0.9, 0.9}
    },
    {
        Type = "StaticModel",
        Name = "room",
        MeshName = "SM_room",
        MaterialName = "M_room",
    },
}

Scene.ObjectRelations = {
}

-- Scene settings
Scene.Settings = {
    BackgroundColor = {100/255, 149/255, 253/255},
    Culling = true
}