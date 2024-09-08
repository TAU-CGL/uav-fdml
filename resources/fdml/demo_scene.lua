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
        Path = "/fdml/old/240521-141038/240521-141038.jpg"
    },
    {
        Name = "T_DIF_drone",
        Path = "/fdml/drone/T_DIF_drone.png"
    },
    {
        Name = "T_SPEC_drone",
        Path = "/fdml/drone/T_MTL_drone.png"
    },
    {
        Name = "T_basilica",
        Path = "/fdml/drone/basilica.png"
    },
}
Scene.Materials = {
    {
        Name = "M_room",
        ShaderName = "S_default",
        -- DiffuseTexture = "T_room",
        Shininess = 1024
    },
    {
        Name = "M_cursor",
        ShaderName = "S_default",
        -- DiffuseColor = {33/255, 114/255, 41/255, 0.3},
        DiffuseColor = {10/255, 10/255, 10/255, 0.4},
        SpecularIntensity = 0.0,
    },
    {
        Name = "M_custom_gizmo",
        ShaderName = "S_custom_gizmo",
    },
    {
        Name = "M_localization",
        ShaderName = "S_default",
        DiffuseColor = {0.0, 1.0, 1.0, 1.0}
    },
    {
        Name = "M_drone",
        ShaderName = "S_default",
        DiffuseTexture = "T_DIF_drone",
        SpecularTexture = "T_SPEC_drone",
        SpecularIntensity = 10.0,
        Shininess = 128,
        Cubemap = "T_basilica",
        ReflectionIntensity = 0.6
    }

}
Scene.StaticMeshes = {
    -- {
    --     Name = "SM_room",
    --     Path = "/fdml/scans/240521-141038/240521-141038-scaled.obj",
    --     KeepData = true
    -- },
    -- {
    --     Name = "SM_room",
    --     Path = "/fdml/scans/isprs/cs4.obj",
    --     KeepData = true
    -- },
    {
        Name = "SM_room",
        Path = "/fdml/scans/perlin/perlin3.obj",
        KeepData = true
    },
    {
        Name = "SM_cursor",
        Path = "/fdml/cursor.obj",
        KeepData = true
    },
    {
        Name = "SM_drone",
        Path = "/fdml/drone/SM_drone.fbx",
    },
}
Scene.SkeletalMeshes = {
}

Scene.Objects = {
    {
        Type = "AmbientLight",
        Name = "ambientLight",
        Intensity = 0.7,
        Color = {1, 0.9, 0.9}
    },
    -- {
    --     Type = "StaticModel",
    --     Name = "room",
    --     MeshName = "SM_room",
    --     MaterialName = "M_room",
    -- },
    {
        Type = "StaticModel",
        Name = "demoDrone",
        MeshName = "SM_drone",
        MaterialName = "M_drone",
        Position = {0, -5, 0},
        Scale = {0.5, 0.5, 0.5},
    },
}

Scene.ObjectRelations = {
}

-- Scene settings
Scene.Settings = {
    BackgroundColor = {100/255, 149/255, 253/255},
    Culling = true
}