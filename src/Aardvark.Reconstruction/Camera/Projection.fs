namespace rec Aardvark.Reconstruction

open Aardvark.Base

[<StructuredFormatDisplay("{AsString}")>]
type Projection =
    {
        focalLength     : float
        aspect          : float
        principalPoint  : V2d
        imageSize       : V2i
        distortion      : Distortion2d
    }

    member private x.AsString =
        sprintf "Projection { focal = %A; aspect = %A; d = %A }"
            x.focalLength
            x.aspect
            x.distortion

    override x.ToString() = x.AsString

    //static member inline (+) (l : Projection, r : Projection) = Projection.compose l r
    //static member inline (-) (l : Projection, r : Projection) = Projection.compose l (Projection.inverse r)

    member x.Trafo = Projection.toTrafo x
    //member x.Inverse = Projection.inverse x

    member x.Project pt = Projection.project pt x
    member x.ProjectUnsafe pt = Projection.projectUnsafe pt x
    member x.Unproject pt = Projection.unproject pt x

module Projection =

    let identity =
        {
            aspect = 1.0
            focalLength = 1.0
            principalPoint = V2d.Zero
            imageSize = V2i.II
            distortion = Distortion2d.Identity
        }

    let project (p : V3d) (proj : Projection) =
        if p.Z >= 0.0 then  
            None
        else
            let dp = proj.distortion.TransformPos (p.XY / -p.Z)
            let rp = proj.focalLength * dp * V2d(1.0, proj.aspect) + proj.principalPoint
            if rp.AllGreaterOrEqual -1.0 && rp.AllSmallerOrEqual 1.0 then
                Some rp
            else    
                None

    let projectUnsafe (p : V3d) (proj : Projection) =
        let dp = proj.distortion.TransformPos (p.XY / -p.Z)
        let rp = proj.focalLength * dp * V2d(1.0, proj.aspect) + proj.principalPoint
        rp

    let unproject (p : V2d) (proj : Projection) =
        let dp = (p - proj.principalPoint) / (V2d(1.0, proj.aspect) * proj.focalLength)
        let p = proj.distortion.InvTransformPos(dp)
        V3d(p, -1.0)


    let toTrafo (proj : Projection) =
        Trafo2d(
            M33d(
                proj.focalLength,           0.0,                                -proj.principalPoint.X,
                0.0,                        proj.focalLength * proj.aspect,     -proj.principalPoint.Y,
                0.0,                        0.0,                                -1.0
            ),
            M33d(
                1.0/proj.focalLength,   0.0,                                     -proj.principalPoint.X / proj.focalLength,
                0.0,                    1.0 / (proj.aspect * proj.focalLength),  -proj.principalPoint.Y / (proj.focalLength * proj.aspect),
                0.0,                    0.0,                                     -1.0
            )
        )

