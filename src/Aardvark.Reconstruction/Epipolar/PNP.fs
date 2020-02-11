namespace Aardvark.Reconstruction

open Aardvark.Base

[<AutoOpen>]
module ``DLT Camera recovery`` =
    // http://www.maths.lth.se/matematiklth/personal/calle/datorseende13/notes/forelas3.pdf

    module P6P = 
        let private PM (obsPoints : (V2d * V3d)[]) =
            let rows = obsPoints.Length * 3
            let cols = 12 + obsPoints.Length
        
            let arr =
                Array.init rows ( fun c ->
                    Array.zeroCreate cols
                )

            for i in 0..obsPoints.Length-1 do
                let (x,X) = obsPoints.[i]
            
                let row1 = arr.[3 * i]
                let row2 = arr.[3 * i + 1]
                let row3 = arr.[3 * i + 2]
            
                row1.[0] <- X.X
                row1.[1] <- X.Y
                row1.[2] <- X.Z
                row1.[3] <- 1.0
                row2.[4] <- X.X
                row2.[5] <- X.Y
                row2.[6] <- X.Z
                row2.[7] <- 1.0
                row3.[8] <- X.X
                row3.[9] <- X.Y
                row3.[10] <- X.Z
                row3.[11] <- 1.0
            
                row1.[12 + i] <- -x.X
                row2.[12 + i] <- -x.Y
                row3.[12 + i] <- -1.0

            let inline v i j = arr.[i].[j]

            Array2D.init rows cols v
       
        /// recovers a Camera from at least 6 2D <> 3D correspondences
        let recover (correspondences : array<V2d * V3d>) =
            if correspondences.Length < 6 then
                None
            else
                let PM = PM correspondences

                match SVD.Decompose PM with
                | None ->
                    None

                | Some(U,S,Vt) ->
            
                    let sol = Array.init (12 + correspondences.Length) ( fun i -> Vt.[(11 + correspondences.Length),i] )
            
                    // https://www.uio.no/studier/emner/matnat/its/UNIK4690/v16/forelesninger/lecture_5_2_pose_from_known_3d_points.pdf

                    let inline s i = sol.[i]
                    let P = 
                        M34d(
                            s 0, s 1, s 2, s 3,
                            s 4, s 5, s 6, s 7,
                            s 8, s  9, s 10, s 11
                        )

                    let M = 
                        M33d(
                            s 0, s 1, s 2,
                            s 4, s 5, s 6,
                            s 8, s 9, s 10
                        )
                
                    let P = P * sign1 M.Det
            
                    let Pi = P.Inverse
                    let pos = Pi.TransformPos V3d.Zero
            
                    let (ka,ra) =
                        // M^-1= (Q * R)
                        // M = (Q * R)^-1
                        // M = (R^-1 * Q^-1)
                        let (Q,R) = QR.Decompose Pi
                        R.Inverse, Q.Transposed

                    let D = 
                        M33d(
                            -sign1 ka.M00, 0.0, 0.0,
                            0.0, -sign1 ka.M11, 0.0,
                            0.0, 0.0, sign1 ka.M22
                        )

                    let ra = D * ra
                    let ka = M33d.op_Explicit ka * D

                    let cv = { trafo = Euclidean3d(ra, ra * -pos)}
                    let ka = ka / -ka.M22
                    let fx = ka.M00
                    let fy = ka.M11
                    let cx = -ka.M02
                    let cy = -ka.M12
                    let proj = { Projection.focalLength = fx; aspect = fy/fx; distortion = { principalPoint = V2d(cx, cy); distortion = RadialDistortion2d.Identity; imageSize = V2i.II } }

                    Some {view = cv; proj = proj}
