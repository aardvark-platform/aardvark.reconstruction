﻿namespace Aardvark.Reconstruction

open Aardvark.Base

#nowarn "49" //uppercase variable identifiers

module FundamentalMatrix =

    [<AutoOpen>]
    module private Helpers = 
        let inline v2dAvg (vs : V2d[]) =
            if vs.Length <= 0 then 
                V2d.NaN
            else
                let mutable sum = V2d.OO
                for v in vs do
                    sum <- sum + v
                sum / float vs.Length
            
        let inline v2dSg (vs : V2d[]) (avg : V2d) =
            let mutable sum = 0.0
            for v in vs do  
                let x = v.X - avg.X
                let y = v.Y - avg.Y
                let b = x * x + y * y
                sum <- sum + b
            sqrt (1.0 / (2.0 * float vs.Length) * sum)

        //Chojnacki, Wojciech, and Michael J. Brooks. "On the consistency of the normalized eight-point algorithm." Journal of Mathematical Imaging and Vision 28.1 (2007): 19-27.
        //Section 3
        let eightpoint (eps : float) (lr : (V2d * V2d)[]) (f_out : byref<M33d>) (e0_out : byref<V2d>) (e1_out : byref<V2d>) =
            if not (lr.Length >= 8) then
                Log.error "too few points (need: 8; got: %d)" lr.Length
                false
            else
                let ls = lr |> Array.map fst
                let rs = lr |> Array.map snd
                
                let ml = ls |> v2dAvg
                let mr = rs |> v2dAvg
        
                let sl = v2dSg ls ml
                let sr = v2dSg rs mr
        
                let lnorms = ls |> Array.map ( fun v -> V2d( (v.X - ml.X) / sl, (v.Y - ml.Y) / sl ) )
                let rnorms = rs |> Array.map ( fun v -> V2d( (v.X - mr.X) / sr, (v.Y - mr.Y) / sr ) )
        
                let Tl = M33d( 1.0/sl, 0.0, -(1.0/sl)*ml.X, 0.0, 1.0/sl, -(1.0/sl)*ml.Y, 0.0, 0.0, 1.0 )
                let Tr = M33d( 1.0/sr, 0.0, -(1.0/sr)*mr.X, 0.0, 1.0/sr, -(1.0/sr)*mr.Y, 0.0, 0.0, 1.0 )
        
                let y (l : V2d) (r : V2d) i =
                    match i with
                    | 0 -> l.X * r.X
                    | 1 -> l.Y * r.X
                    | 2 -> r.X
                    | 3 -> l.X * r.Y
                    | 4 -> l.Y * r.Y
                    | 5 -> r.Y
                    | 6 -> l.X
                    | 7 -> l.Y
                    | 8 -> 1.0
                    | _ -> failwith "no"

                let M = Array2D.init 9 lr.Length ( fun r c -> y lnorms.[c] rnorms.[c] r )

                match SVD.Decompose M with
                | None -> false
                | Some(U,md,Vt) ->
                    let idx = if lr.Length > 8 then 8 else 7
                    let mi = md.[idx,idx]
                    if Fun.IsTiny(mi, eps) |> not then
                        false
                    else

                        let Fals = M33d U.[*,8]
                        let U = ()
                        let Fals = Tr.Transposed * Fals * Tl 

                        match SVD.Decompose Fals with
                        | None -> false
                        | Some (U,S,Vt2) ->

                            let mutable S = S
                            S.[2,2] <- 0.0
                            let Fhrt = U * S * Vt2
            
                            let e1 = if abs U.M22 < eps then V2d.Zero else U.C2.XY / U.M22  
                            let e0 = if abs Vt2.M22 < eps then V2d.Zero else Vt2.R2.XY / Vt2.M22

                            f_out <- Fhrt
                            e0_out <- e0
                            e1_out <- e1
                
                            true

    let algebraicError (lr : (V2d * V2d)[]) (F : M33d) =
        if lr.Length = 0 then
            0.0
        else
            lr |> Array.averageBy (fun (l,r) ->
                let l = V3d(l,1.0)
                let r = V3d(r,1.0)
                let p = F * l
                if Fun.ApproximateEquals(p.XY, V2d.Zero) then
                    let err = Vec.dot r p
                    err * err
                else
                    let err = Vec.dot r (p / p.XY.Length)
                    err * err
            )
                |> sqrt

    let recover (eps : float) (lr : (V2d * V2d)[]) =
        let mutable F = Unchecked.defaultof<_>
        let mutable e0 = Unchecked.defaultof<_>
        let mutable e1 = Unchecked.defaultof<_>
        let succ = eightpoint eps lr &F &e0 &e1
        if succ then
            (F, e0, e1) |> Some
        else
            None
            
    let inline approxEquals eps (F0 : M33d) (F1 : M33d) =
        let s = F0.M00/F1.M00
        Fun.ApproximateEquals(F0,F1*s,eps)

    // Zhang Ch.9
    let fromCameras (l : Camera) (r : Camera) =
        let mot = r - l |> CameraMotion.normalize
        let R : M33d = mot.trafo.Rot |> Rot3d.op_Explicit
        let t = mot.trafo.Trans
        let Kl = l.proj.Trafo
        let Kr = r.proj.Trafo
        let F = Kr.Backward.Transposed * R * Kl.Forward.Transposed * M33d.crossProductMatrix (Kl.Forward * R.Transposed * t)
        let lsbr = r.ProjectUnsafe l.Location
        let rsbl = l.ProjectUnsafe r.Location
        F,lsbr,rsbl

    //Terzakis, George. Relative camera pose recovery and scene reconstruction with the essential matrix in a nutshell. Technical report MIDAS. SNMSE. 2013. TR. 007, Marine and Industrial Dynamic Analysis School of Marine Science and Engineering, Plymouth University, Tech. Rep, 2013.    
    let decompose2 (F : M33d) (lIntern : Projection) (rIntern : Projection) (referencePoints : list<V2d * V2d>)  =
        let Kl = Projection.toTrafo lIntern
        let Kr = Projection.toTrafo rIntern

        let ess = Kr.Forward.Transposed * F * Kl.Forward
        match SVD.Decompose ess with
        | Some(U,S,Vt) -> 
            let En = U * M33d.FromDiagonal(1.0,1.0,0.0) * Vt
            let eTe = En.Transposed * En
            
            let inline e i j =    En.[i-1,j-1]
            let inline ete i j = min eTe.[i-1,j-1] 1.0


            let mutable b1 = sqrt (1.0 - ete 1 1)
            let mutable b2 = sqrt (1.0 - ete 2 2)
            let mutable b3 = sqrt (1.0 - ete 3 3)

            if b1 > b2 && b1 > b3 then
                if ete 1 2 > 0.0 then
                    b2 <- -b2
                if ete 1 3 > 0.0 then
                    b3 <- -b3
            elif b2 > b1 && b2 > b3 then
                if ete 1 2 > 0.0 then
                    b1 <- -b1
                if ete 2 3 > 0.0 then
                    b3 <- -b3
            else
                if ete 1 3 > 0.0 then
                    b1 <- -b1
                if ete 2 3 > 0.0 then
                    b2 <- -b2

            let t1 = V3d( b1, b2, b3) |> Vec.normalize
            let t2 = V3d(-b1,-b2,-b3) |> Vec.normalize                            

            let C = 
                M33d(
                      e 2 2 * e 3 3 - e 3 2 * e 2 3,  -(e 2 1 * e 3 3 - e 3 1 * e 2 3),   e 2 1 * e 3 2 - e 3 1 * e 2 2,
                    -(e 1 2 * e 3 3 - e 3 2 * e 1 3),   e 1 1 * e 3 3 - e 3 1 * e 1 3,  -(e 1 1 * e 3 2 - e 3 1 * e 1 2),
                      e 1 2 * e 2 3 - e 2 2 * e 1 3,  -(e 1 1 * e 2 3 - e 2 1 * e 1 3),   e 1 1 * e 2 2 - e 2 1 * e 1 2                    
                )

            let R1 = (C.Transposed + M33d.crossProductMatrix(t1) * En.Transposed).ToOrthoNormal().Transposed    
            let R2 = (C.Transposed + M33d.crossProductMatrix(t2) * En.Transposed).ToOrthoNormal().Transposed           

            [
                { trafo = Euclidean3d(Rot3d.FromM33d R1, R1 * t1); isNormalized = true }
                { trafo = Euclidean3d(Rot3d.FromM33d R1, R1 * t2); isNormalized = true }
                { trafo = Euclidean3d(Rot3d.FromM33d R2, R2 * t1); isNormalized = true }
                { trafo = Euclidean3d(Rot3d.FromM33d R2, R2 * t2); isNormalized = true }
            ]

        | None -> 
            []
        

    // gelbes book S.178
    let decompose (F : M33d) (lIntern : Projection) (rIntern : Projection) (referencePoints : list<V2d * V2d>)  =
        let Kl = Projection.toTrafo lIntern
        let Kr = Projection.toTrafo rIntern

        let ess = Kr.Forward.Transposed * F * Kl.Forward
        
        match SVD.Decompose ess with
        | Some(U,S,Vt) -> 
            let G = 
                M33d(0.0,  1.0, 0.0, 
                    -1.0,  0.0, 0.0, 
                     0.0,  0.0, 1.0)

            let R1 = U * G * Vt
            let R2 = U * G.Transposed * Vt
        
            let inline t (m : M33d) =
                V3d(m.M21,m.M02,m.M10)
        
            let t1 = U * G * S * U.Transposed               |> t
            let t2 = U * G.Transposed * S * U.Transposed    |> t
        
            let R1 = R1.ToOrthoNormal()
            let R2 = R2.ToOrthoNormal()
            let t1 = t1 |> Vec.normalize
            let t2 = t2 |> Vec.normalize

            let check (l : list<CameraMotion>) (R : M33d, t : V3d) =
                try
                    let m = { trafo = Euclidean3d(Rot3d.FromM33d R, t); isNormalized = true }

                    match referencePoints with
                        | [] ->
                            m :: l
                        | _ -> 
                            let lc = { view = CameraView.identity; proj = lIntern }
                            let rc = { view = lc.view + m; proj = rIntern }
                            let valid = 
                                referencePoints |> List.forall (fun (l,r) ->
                                    let lr = Camera.unproject l lc
                                    let rr = Camera.unproject r rc


                                    if Vec.length (Vec.cross lr.Direction rr.Direction) > 0.01 then 
                                        let w = lr.GetMiddlePoint rr

                                        let tl = lr.GetTOfProjectedPoint w
                                        let tr = rr.GetTOfProjectedPoint w

                                        if tl >= 0.0 && tr >= 0.0 then
                                            true
                                        else
                                            false
                                    else
                                        true

                                )
                        
                            if valid then m :: l
                            else l
                    with _ ->
                        l

            List.fold check [] [
                R1, t1
                R2, t1
                R1, t2
                R2, t2
            ] 
        | None -> decompose2 F lIntern rIntern referencePoints
 