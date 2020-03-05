namespace rec Aardvark.Reconstruction

open Aardvark.Base

[<StructuredFormatDisplay("{AsString}")>]
type CameraView =
    {
        trafo : Euclidean3d
    }

    member private x.AsString =
        sprintf "CameraView { location = %A; forward = %A; right = %A; up = %A }"
            x.Location
            x.Forward
            x.Right
            x.Up

    override x.ToString() = x.AsString

    static member inline (+) (c : CameraView, m : CameraMotion) = CameraView.move m c
    static member inline (-) (c : CameraView, m : CameraMotion) = CameraView.move (CameraMotion.inverse m) c
    static member inline (-) (l : CameraView, r : CameraView) = CameraView.motion r l

    member x.Location = CameraView.location x
    member x.Forward = CameraView.forward x
    member x.Right = CameraView.right x
    member x.Up = CameraView.up x
    member x.ViewTrafo = CameraView.viewTrafo x

    member x.Transformed (sim : Similarity3d) = 
        let rot = x.trafo.Rot * sim.Rot.Inverse
        {
            trafo = Euclidean3d(rot, -rot.Transform(sim.TransformPos(x.Location)))
        }

module CameraView =

    let sameness (l : CameraView) (r : CameraView) =
        let dd = (l.Location - r.Location).Length

        let df = abs <| (1.0 - Vec.dot l.Forward r.Forward)
        let du = abs <| (1.0 - Vec.dot l.Up r.Up          )
        let dr = abs <| (1.0 - Vec.dot l.Right r.Right    )
        let da = max df (max du dr) / 2.0

        let aweight = max 1.0 (dd * 10.0)
        dd + aweight * da

    let approxEqual (eps : float) (l : CameraView) (r : CameraView) =
        sameness l r < eps
        //(Rot3d.ApproxEqual(l.trafo.Rot, r.trafo.Rot, eps) || Rot3d.ApproxEqual(-l.trafo.Rot, r.trafo.Rot, eps)) &&
        //(V3d.ApproxEqual(l.trafo.Trans, r.trafo.Trans, eps) || V3d.ApproxEqual(-l.trafo.Trans, r.trafo.Trans, eps))


    // forward = -z
    // f = 1
    // a = 1
    let identity =
        {
            trafo = Euclidean3d.Identity
        }

    let forward (c : CameraView) = c.trafo.InvTransformDir(-V3d.OOI)
    let right (c : CameraView) = c.trafo.InvTransformDir(V3d.IOO)
    let up (c : CameraView) = c.trafo.InvTransformDir(V3d.OIO)
    let location (c : CameraView) = c.trafo.Rot.InvTransform(-c.trafo.Trans)

    let inline transformed (sim : Similarity3d) (c : CameraView) = c.Transformed sim

    let lookAt (eye : V3d) (center : V3d) (sky : V3d) =
        let fv = Vec.normalize (center - eye)
        let rv = Vec.cross fv sky |> Vec.normalize
        let uv = Vec.cross rv fv |> Vec.normalize
        
        //  r => IOO
        //  u => OIO
        // -f => OOI
        // -r.TransformPos(eye) = r.Trans;

        let r = Rot3d.FromFrame(rv, uv, -fv).Inverse
        let t = -r.Transform(eye)
        let e = Euclidean3d(r, t)
        { trafo = e }

    let viewTrafo (c : CameraView) =
        Trafo3d(
            Euclidean3d.op_Explicit c.trafo,
            Euclidean3d.op_Explicit c.trafo.Inverse
        )

    let motion (l : CameraView) (r : CameraView) =
        { 
            trafo = r.trafo * l.trafo.Inverse
            isNormalized = false
        }
    
    let move (motion : CameraMotion) (cam : CameraView) =
        { cam with trafo = motion.trafo * cam.trafo }


[<StructuredFormatDisplay("{AsString}")>]
type CameraMotion =
    {
        trafo : Euclidean3d
        isNormalized : bool
    }

    member private x.AsString =
        let r = x.trafo.Rot.GetEulerAngles() * Constant.DegreesPerRadian
        let t = x.trafo.Trans
        sprintf "CameraMotion { rot = [%.3f°, %.3f°, %.3f°]; trans = %A }" r.X r.Y r.Z t

    override x.ToString() = x.AsString


    member inline x.Normalized = CameraMotion.normalize x
    member inline x.Inverse = CameraMotion.inverse x
    
    static member inline (~-) (m : CameraMotion) = CameraMotion.inverse m
    static member inline (*) (m : CameraMotion, f : float) = CameraMotion.scale f m
    static member inline (*) (f : float, m : CameraMotion) = CameraMotion.scale f m
    static member inline (+) (l : CameraMotion, r : CameraMotion) = CameraMotion.compose l r
    static member Zero = CameraMotion.zero

module CameraMotion =
    
    let zero = { trafo = Euclidean3d.Identity; isNormalized = false }

    let approxEqual (eps : float) (l : CameraMotion) (r : CameraMotion) =
        (Fun.ApproximateEquals(l.trafo.Rot, r.trafo.Rot, eps) || Fun.ApproximateEquals(-l.trafo.Rot, r.trafo.Rot, eps)) &&
        Fun.ApproximateEquals(l.trafo.Trans, r.trafo.Trans, eps)

    let max3 x y z =
        if x > y then   
            if x > z then
                x
            else
                z
        else
            if y > z then
                y
            else
                z

    let similarity (l : CameraMotion) (r : CameraMotion) =
        
        let dp = ((l.trafo.Trans.Normalized) - r.trafo.Trans.Normalized)
        let ddx = abs dp.X
        let ddy = abs dp.Y
        let ddz = abs dp.Z
        
        let aD = max3 ddx ddy ddz

        let x = V3d.IOO
        let y = V3d.OIO
        let z = V3d.OOI
        let px = r.trafo.Rot.InvTransform( l.trafo.Rot.Transform x)
        let py = r.trafo.Rot.InvTransform( l.trafo.Rot.Transform y)
        let pz = r.trafo.Rot.InvTransform( l.trafo.Rot.Transform z)

        let dx = Vec.length(x-px)
        let dy = Vec.length(y-py)
        let dz = Vec.length(z-pz)
        
        let aA = max3 dx dy dz

        let a = max aD aA

        1.0 - a / 2.0


    let normalize (m : CameraMotion) =
        if m.isNormalized then
            m
        elif Fun.ApproximateEquals(m.trafo.Trans, V3d.Zero, 1E-5) then
            { m with isNormalized = true }
        else
            { 
                trafo = Euclidean3d(m.trafo.Rot, Vec.normalize m.trafo.Trans)
                isNormalized = true
            }
            
    let scale (factor : float) (m : CameraMotion) =
        { 
            trafo = Euclidean3d(m.trafo.Rot, factor * m.trafo.Trans)
            isNormalized = false
        }

    let compose (l : CameraMotion) (r : CameraMotion) =
        {
            trafo = r.trafo * l.trafo
            isNormalized = false
        }

    let inverse (m : CameraMotion) =
        { m with trafo = m.trafo.Inverse }

    let ofRotationAndTrans (m : M33d) (trans : V3d) =
        let r = Rot3d.FromM33d m
        { trafo = Euclidean3d(r, r.Transform(trans)); isNormalized = false }