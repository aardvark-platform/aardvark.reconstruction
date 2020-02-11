namespace Aardvark.Reconstruction

open Aardvark.Base

type RadialDistortion2d (forward : float[], backward : float[]) =
    static let invert4 (k1 : float) (k2 : float) (k3 : float) (k4 : float) =
        let k1_2 = k1   * k1
        let k1_3 = k1_2 * k1
        let k1_4 = k1_3 * k1
        let k1_5 = k1_4 * k1
        let k1_6 = k1_5 * k1
        let k1_7 = k1_6 * k1
        let k1_8 = k1_7 * k1
        let k1_9 = k1_8 * k1
        let k2_2 = k2   * k2
        let k2_3 = k2_2 * k2
        let k2_4 = k2_3 * k2
        let k3_2 = k3   * k3
        let k3_3 = k3_2 * k3
        let k4_2 = k4   * k4

        [|
            // b1
            -k1
            
            // b2
            3.0*k1_2 - k2
            
            // b3
            -12.0*k1_3 + 8.0*k1*k2 - k3
            
            // b4
            55.0*k1_4 - 55.0*k1_2*k2 + 5.0*k2_2 + 10.0*k1*k3 - k4
            
            // b5
            -273.0*k1_5 + 364.0*k1_3*k2 - 78.0*k1*k2_2 - 78.0*k1_2*k3 + 12.0*k2*k3 + 12.0*k1*k4
            
            // b6
            1428.0*k1_6 - 2380.0*k1_4*k2 + 840.0*k1_2*k2_2 - 35.0*k2_3 + 560.0*k1_3*k3 - 210.0*k1*k2*k3 + 7.0*k3_2 - 105.0*k1_2*k4 + 14.0*k2*k4
            
            // b7
            -7752.0*k1_7 + 15504.0*k1_5*k2 - 7752.0*k1_3*k2_2 + 816.0*k1*k2_3 - 3876.0*k1_4*k3 + 2448.0*k1_2*k2*k3 - 136.0*k2_2*k3 - 136.0*k1*k3_2
            + 816.0*k1_3*k4 - 272.0*k1*k2*k4 + 16.0*k3*k4

            // b8
            43263.0*k1_8 - 100947.0*k1_6*k2 + 65835.0*k1_4*k2_2 - 11970.0*k1_2*k2_3 + 
            285.0*k2_4 + 26334.0*k1_5*k3 - 23940.0*k1_3*k2*k3 + 3420.0*k1*k2_2*k3 + 1710.0*k1_2*k3_2 - 
            171.0*k2*k3_2 - 5985.0*k1_4*k4 + 3420.0*k1_2*k2*k4 - 171.0*k2_2*k4 - 342.0*k1*k3*k4 + 9.0*k4_2

            // b9
            -246675.0*k1_9 + 657800.0*k1_7*k2 - 531300.0*k1_5*k2_2 + 141680.0*k1_3*k2_3 - 8855.0*k1*k2_4 - 177100.0*k1_6*k3 + 
            212520.0*k1_4*k2*k3 - 53130.0*k1_2*k2_2*k3 + 1540.0*k2_3*k3 - 17710.0*k1_3*k3_2 + 4620.0*k1*k2*k3_2 - 70.0*k3_3 + 42504.0*k1_5*k4 - 
            35420.0*k1_3*k2*k4 + 4620.0*k1*k2_2*k4 + 4620.0*k1_2*k3*k4 - 420.0*k2*k3*k4 - 210.0*k1*k4_2

        |]

    static let invert3 (k1 : float) (k2 : float) (k3 : float) =
        let k1_2 = k1   * k1
        let k1_3 = k1_2 * k1
        let k1_4 = k1_3 * k1
        let k1_5 = k1_4 * k1
        let k1_6 = k1_5 * k1
        let k1_7 = k1_6 * k1
        let k1_8 = k1_7 * k1
        let k1_9 = k1_8 * k1
        let k2_2 = k2   * k2
        let k2_3 = k2_2 * k2
        let k2_4 = k2_3 * k2
        let k3_2 = k3   * k3
        let k3_3 = k3_2 * k3

        [|
            // b1
            -k1
            
            // b2
            3.0*k1_2 - k2
            
            // b3
            -12.0*k1_3 + 8.0*k1*k2 - k3
            
            // b4
            55.0*k1_4 - 55.0*k1_2*k2 + 5.0*k2_2 + 10.0*k1*k3 
            
            // b5
            -273.0*k1_5 + 364.0*k1_3*k2 - 78.0*k1*k2_2 - 78.0*k1_2*k3 + 12.0*k2*k3
            
            // b6
            1428.0*k1_6 - 2380.0*k1_4*k2 + 840.0*k1_2*k2_2 - 35.0*k2_3 + 560.0*k1_3*k3 - 210.0*k1*k2*k3 + 7.0*k3_2
            
            // b7
            -7752.0*k1_7 + 15504.0*k1_5*k2 - 7752.0*k1_3*k2_2 + 816.0*k1*k2_3 - 3876.0*k1_4*k3 + 2448.0*k1_2*k2*k3 - 136.0*k2_2*k3 - 136.0*k1*k3_2

            // b8
            43263.0*k1_8 - 100947.0*k1_6*k2 + 65835.0*k1_4*k2_2 - 11970.0*k1_2*k2_3 + 
            285.0*k2_4 + 26334.0*k1_5*k3 - 23940.0*k1_3*k2*k3 + 3420.0*k1*k2_2*k3 + 1710.0*k1_2*k3_2 - 
            171.0*k2*k3_2

            // b9
            -246675.0*k1_9 + 657800.0*k1_7*k2 - 531300.0*k1_5*k2_2 + 141680.0*k1_3*k2_3 - 8855.0*k1*k2_4 - 177100.0*k1_6*k3 + 
            212520.0*k1_4*k2*k3 - 53130.0*k1_2*k2_2*k3 + 1540.0*k2_3*k3 - 17710.0*k1_3*k3_2 + 4620.0*k1*k2*k3_2 - 70.0*k3_3
        |]

    static let invert2 (k1 : float) (k2 : float) =
        let k1_2 = k1   * k1
        let k1_3 = k1_2 * k1
        let k1_4 = k1_3 * k1
        let k1_5 = k1_4 * k1
        let k1_6 = k1_5 * k1
        let k1_7 = k1_6 * k1
        let k1_8 = k1_7 * k1
        let k1_9 = k1_8 * k1
        let k2_2 = k2   * k2
        let k2_3 = k2_2 * k2
        let k2_4 = k2_3 * k2

        [|
            // b1
            -k1
            
            // b2
            3.0*k1_2 - k2
            
            // b3
            -12.0*k1_3 + 8.0*k1*k2
            
            // b4
            55.0*k1_4 - 55.0*k1_2*k2 + 5.0*k2_2
            
            // b5
            -273.0*k1_5 + 364.0*k1_3*k2 - 78.0*k1*k2_2
            
            // b6
            1428.0*k1_6 - 2380.0*k1_4*k2 + 840.0*k1_2*k2_2 - 35.0*k2_3
            
            // b7
            -7752.0*k1_7 + 15504.0*k1_5*k2 - 7752.0*k1_3*k2_2 + 816.0*k1*k2_3

            // b8
            43263.0*k1_8 - 100947.0*k1_6*k2 + 65835.0*k1_4*k2_2 - 11970.0*k1_2*k2_3

            // b9
            -246675.0*k1_9 + 657800.0*k1_7*k2 - 531300.0*k1_5*k2_2 + 141680.0*k1_3*k2_3 - 8855.0*k1*k2_4
        |]

    static let invert1 (k1 : float) =
        let k1_2 = k1   * k1
        let k1_3 = k1_2 * k1
        let k1_4 = k1_3 * k1
        let k1_5 = k1_4 * k1
        let k1_6 = k1_5 * k1
        let k1_7 = k1_6 * k1
        let k1_8 = k1_7 * k1
        let k1_9 = k1_8 * k1

        [|
            // b1
            -k1
            
            // b2
            3.0*k1_2
            
            // b3
            -12.0*k1_3
            
            // b4
            55.0*k1_4
            
            // b5
            -273.0*k1_5
            
            // b6
            1428.0*k1_6
            
            // b7
            -7752.0*k1_7

            // b8
            43263.0*k1_8

            // b9
            -246675.0*k1_9
        |]

    static let rec trimTrailing (i : int) (a : float[]) =
        if i < 0 then 
            [||]
        else
            if a.[i] = 0.0 then
                trimTrailing (i-1) a
            else
                Array.take (i + 1) a
    
    static let identity = RadialDistortion2d([||], [||])

    member x.ForwardCoefficients = forward
    member x.BackwardCoefficients = backward
    
    member x.K1 = if forward.Length > 0 then forward.[0] else 0.0
    member x.K2 = if forward.Length > 1 then forward.[1] else 0.0
    member x.K3 = if forward.Length > 2 then forward.[2] else 0.0
    member x.K4 = if forward.Length > 3 then forward.[3] else 0.0

    override x.ToString() =
        let ks = forward |> Seq.mapi (fun i k -> sprintf "k%d=%.3g" (i + 1) k) |> String.concat "; "
        sprintf "RDist { %s }" ks

    member x.Inverse = RadialDistortion2d(backward, forward)

    new(k1 : float, k2 : float, k3 : float, k4 : float) =
        let forward = trimTrailing 3 [| k1; k2; k3; k4 |]
        let backward = (invert4 k1 k2 k3 k4)
        RadialDistortion2d(forward, backward)
    
    new(k1 : float, k2 : float, k3 : float) =
        let forward = trimTrailing 2 [| k1; k2; k3|]
        let backward = (invert3 k1 k2 k3)
        RadialDistortion2d(forward, backward)
    
    new(k1 : float, k2 : float) =
        let forward = trimTrailing 1 [| k1; k2|]
        let backward = (invert2 k1 k2)
        RadialDistortion2d(forward, backward)
    
    new(k1 : float) =
        let forward = trimTrailing 0 [| k1 |]
        let backward = (invert1 k1)
        RadialDistortion2d(forward, backward)

    new() = 
        RadialDistortion2d([||], [||])

    member x.TransformPos(p : V2d) =
        let r2 = p.LengthSquared
        let struct (_,factor) = forward |> Array.fold (fun (struct (rc,s)) ki -> struct (r2 * rc, s + ki * rc)) (struct (r2, 1.0))
        p * factor
        
    member x.InvTransformPos(p : V2d) =
        let r2 = p.LengthSquared
        let struct (_,factor) = backward |> Array.fold (fun (struct (rc,s)) ki -> struct (r2 * rc, s + ki * rc)) (struct (r2, 1.0))
        p * factor

    member x.IsIdentity = forward.Length = 0

    static member Identity = identity
    

type Distortion =
    {
        principalPoint  : V2d // ndc
        imageSize       : V2i // pixels
        distortion      : RadialDistortion2d
    }

module Distortion =

    let empty = { principalPoint = V2d.Zero; imageSize = V2i.II; distortion = RadialDistortion2d.Identity }

    let sameness (l : Distortion) (r : Distortion) =
        if l.distortion.ForwardCoefficients.Length = r.distortion.ForwardCoefficients.Length then
            V2d.Distance(l.principalPoint, r.principalPoint) +
            //V2i.Distance(l.imageSize, r.imageSize) +
            (Array.zip l.distortion.ForwardCoefficients r.distortion.ForwardCoefficients
                |> Array.sumBy (fun (l,r) -> abs (l-r)))
        else
            1000.0 //?
    let approxEqual (eps : float) (l : Distortion) (r : Distortion) =
        V2d.ApproxEqual(l.principalPoint, r.principalPoint, eps) &&
        l.imageSize = r.imageSize &&
        l.distortion.ForwardCoefficients.Length = r.distortion.ForwardCoefficients.Length &&
        Array.forall2 (fun lc rc -> abs (rc - lc) <= eps) l.distortion.ForwardCoefficients r.distortion.ForwardCoefficients

    let distort (ndc : V2d) (dist : Distortion) =
        let d = ndc + dist.principalPoint
        let px = 0.5 * d * V2d dist.imageSize
        let pxd = dist.distortion.TransformPos px
        2.0 * pxd / V2d dist.imageSize

    let undistort (ndc : V2d) (dist : Distortion) =
        let pxd = 0.5 * ndc * V2d dist.imageSize
        let px = dist.distortion.InvTransformPos pxd
        let d = 2.0 * px / V2d dist.imageSize
        d - dist.principalPoint
