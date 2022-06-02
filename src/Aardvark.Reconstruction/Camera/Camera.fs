namespace rec Aardvark.Reconstruction

open Aardvark.Base

[<StructuredFormatDisplay("{AsString}")>]
type Camera = { view : CameraView; proj : Projection} with
    
    member private x.AsString =
        sprintf "Camera { view = %A; proj = %A }"
            x.view
            x.proj

    override x.ToString() = x.AsString
    
    static member inline (+) (c : Camera, m : CameraMotion) = Camera.move m c
    static member inline (-) (c : Camera, m : CameraMotion) = Camera.move (CameraMotion.inverse m) c
    static member inline (-) (l : Camera, r : Camera) = Camera.motion r l

    member x.Location = Camera.location x
    member x.Forward = Camera.forward x
    member x.Right = Camera.right x
    member x.Up = Camera.up x
    member x.ViewTrafo = Camera.viewTrafo x
    member x.ProjTrafo(near, far) = Camera.projTrafo near far
    member x.ViewProjTrafo(near, far) = Camera.viewProjTrafo near far

    member x.Project pt = Camera.project pt x
    member x.ProjectUnsafe pt = Camera.projectUnsafe pt x
    member x.Unproject pt = Camera.unproject pt x

module Camera =

    let location (c : Camera) = CameraView.location c.view
    let forward (c : Camera) = CameraView.forward c.view
    let right (c : Camera) = CameraView.right c.view
    let up (c : Camera) = CameraView.up c.view


    let projTrafo (near : float) (far : float) (cam : Camera) =
        let proj = cam.proj
        let tnf = 2.0 * far * near
        let a = (far + near) / (far - near)
        let b = tnf / (near - far)


        // (near * a + b) / near = -1
        // (far * a + b) / far = 1

        // a + b / near = -1
        // a + b / far = 1
        
        // near * a + b = -near     * far
        // far * a + b = far        * near

        // near * a + b = -near   
        // far * a + b = far      
        // a = (near + far) / (far - near)

        // far * b - near * b = -2*near*far
        // b = 2*near*far / (near - far)

        Trafo3d(
            M44d(
                proj.focalLength,           0.0,                               -proj.principalPoint.X,  0.0,
                0.0,                        proj.focalLength * proj.aspect,    -proj.principalPoint.Y,  0.0,
                0.0,                        0.0,                               -a,                      b,
                0.0,                        0.0,                               -1.0,                    0.0
            ),
            M44d(
                1.0/proj.focalLength,   0.0,                                    0.0,       -proj.principalPoint.X / proj.focalLength,
                0.0,                    1.0 / (proj.aspect * proj.focalLength), 0.0,       -proj.principalPoint.Y / (proj.focalLength * proj.aspect),
                0.0,                    0.0,                                    0.0,       -1.0,
                0.0,                    0.0,                                    1.0/b,     -a/b

            )
        )
        
    let viewTrafo (c : Camera) =
        CameraView.viewTrafo c.view

    let viewProjTrafo (near : float) (far : float) (c : Camera) =
        CameraView.viewTrafo c.view * projTrafo near far c
        
    let projectUnsafe (pt : V3d) (c : Camera) =
        let viewSpace = c.view.trafo.TransformPos pt
        Projection.projectUnsafe viewSpace c.proj
        
    let project (pt : V3d) (c : Camera) =
        let viewSpace = c.view.trafo.TransformPos pt
        Projection.project viewSpace c.proj
        
    let unproject (pt : V2d) (c : Camera) =
        let p = Projection.unproject pt c.proj
        let pp = c.view.trafo.InvTransformDir p
        let dir = Vec.normalize pp
        let c = CameraView.location c.view
        Ray3d(c, dir)

    let move (motion : CameraMotion) (c : Camera) =
        { c with view = CameraView.move motion c.view }

    let motion (l : Camera) (r : Camera) =
        CameraView.motion l.view r.view
