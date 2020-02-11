namespace rec Aardvark.Reconstruction

open Aardvark.Base

[<StructuredFormatDisplay("{AsString}")>]
type Projection =
    {
        focalLength     : float
        aspect          : float
        distortion      : Distortion
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
            distortion = { imageSize = V2i.II; distortion = RadialDistortion2d.Identity; principalPoint = V2d.Zero }
        }

    let sameness (l : Projection) (r : Projection) =
        abs (l.focalLength - r.focalLength) +
        abs (l.aspect - r.aspect) +
        Distortion.sameness l.distortion r.distortion

    let approxEqual (eps : float) (l : Projection) (r : Projection) =
        Fun.ApproximateEquals(l.focalLength, r.focalLength, eps) &&
        Fun.ApproximateEquals(l.aspect, r.aspect, eps) &&
        Distortion.approxEqual eps l.distortion r.distortion

    let project (p : V3d) (proj : Projection) =
        let z = -p.Z
        let rp = proj.focalLength * p.XY * V2d(1.0, proj.aspect)
        let ptest = (rp + proj.distortion.principalPoint * z)
        if ptest.X >= -z && ptest.X <= z && ptest.Y >= -z && ptest.Y <= z && z > 0.0 then
            let p = (rp / z)
            Distortion.distort p proj.distortion |> Some
        else
            None

    let projectUnsafe (p : V3d) (proj : Projection) =
        let p = (proj.focalLength * p.XY * V2d(1.0, proj.aspect)) / -p.Z 
        Distortion.distort p proj.distortion
      

    let unproject (p : V2d) (proj : Projection) =
        let p = Distortion.undistort p proj.distortion
        // p.Z = -1
        // (w - proj.principalPoint) / (proj.focalLength * V2d(1.0, proj.aspect)) = p.XY
        V3d(p / (proj.focalLength * V2d(1.0, proj.aspect)), -1.0)


    let toTrafo (proj : Projection) =
        Trafo2d(
            M33d(
                proj.focalLength,           0.0,                                -proj.distortion.principalPoint.X,
                0.0,                        proj.focalLength * proj.aspect,     -proj.distortion.principalPoint.Y,
                0.0,                        0.0,                                -1.0
            ),
            M33d(
                1.0/proj.focalLength,   0.0,                                     -proj.distortion.principalPoint.X / proj.focalLength,
                0.0,                    1.0 / (proj.aspect * proj.focalLength),  -proj.distortion.principalPoint.Y / (proj.focalLength * proj.aspect),
                0.0,                    0.0,                                     -1.0
            )
        )

