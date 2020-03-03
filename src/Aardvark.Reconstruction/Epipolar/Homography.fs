namespace Aardvark.Reconstruction

open Aardvark.Base

module Homography =

    [<AutoOpen>]
    module private Helpers = 
        let PM (p1 : V2d) (p2 : V2d) (p3 : V2d) (p4 : V2d) (p1' : V2d) (p2' : V2d) (p3' : V2d) (p4' : V2d) =
            let x1 = p1.X
            let y1 = p1.Y
            let x2 = p2.X 
            let y2 = p2.Y 
            let x3 = p3.X
            let y3 = p3.Y
            let x4 = p4.X 
            let y4 = p4.Y
            let x1' = p1'.X
            let y1' = p1'.Y
            let x2' = p2'.X 
            let y2' = p2'.Y 
            let x3' = p3'.X
            let y3' = p3'.Y
            let x4' = p4'.X 
            let y4' = p4'.Y
        

            let arr =
                [|
                    [| -x1; -y1; -1.0; 0.0; 0.0; 0.0; x1*x1'; y1*x1'; x1' |]
                    [| 0.0; 0.0; 0.0; -x1; -y1; -1.0; x1*y1'; y1*y1'; y1' |]
                    [| -x2; -y2; -1.0; 0.0; 0.0; 0.0; x2*x2'; y2*x2'; x2' |]
                    [| 0.0; 0.0; 0.0; -x2; -y2; -1.0; x2*y2'; y2*y2'; y2' |]
                    [| -x3; -y3; -1.0; 0.0; 0.0; 0.0; x3*x3'; y3*x3'; x3' |]
                    [| 0.0; 0.0; 0.0; -x3; -y3; -1.0; x3*y3'; y3*y3'; y3' |]
                    [| -x4; -y4; -1.0; 0.0; 0.0; 0.0; x4*x4'; y4*x4'; x4' |]
                    [| 0.0; 0.0; 0.0; -x4; -y4; -1.0; x4*y4'; y4*y4'; y4' |]
                    [| 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 1.0        |]
                |]
        
            let inline value r c = arr.[r].[c]
        
            Array2D.init 9 9 value

        let PMLsq (ps : (V2d * V2d)[]) =
    
            let arr =
                ps |> Array.collect (fun (l,r) ->
                    [|
                        [| -l.X; -l.Y; -1.0; 0.0; 0.0; 0.0; l.X*r.X; l.Y*r.X; r.X |]
                        [| 0.0; 0.0; 0.0; -l.X; -l.Y; -1.0; l.X*r.Y; l.Y*r.Y; r.Y |]
                    |]
                )
        
            let inline value r c = arr.[r].[c]
            Array2D.init (ps.Length * 2) 9 value

        let solve (lr0 : V2d*V2d) (lr1 : V2d*V2d) (lr2 : V2d*V2d) (lr3 : V2d*V2d) =
        
            let p1 = lr0 |> fst
            let p2 = lr1 |> fst
            let p3 = lr2 |> fst
            let p4 = lr3 |> fst
        
            let p1r = lr0 |> snd
            let p2r = lr1 |> snd
            let p3r = lr2 |> snd
            let p4r = lr3 |> snd

            let PM = PM p1 p2 p3 p4 p1r p2r p3r p4r
            let R = [| 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 1.0 |]
     
            try
                let perm = PM.LuFactorize()
                let pp = PM.LuSolve(perm,R) // <- sometimes broken??
                Some (M33d pp)
            with
                |_-> None
   
        let estimate (lr : (V2d * V2d)[]) =
            let PM = PMLsq lr
            match SVD.Decompose(PM) with
                | Some (U, S, Vt) ->
                    let pp = Array.init 9 (fun i -> Vt.[8,i])
                    Some (M33d pp)
                | _ -> 
                    None

    let recover (corresponding : array<V2d * V2d>) =
        if corresponding.Length < 4 then
            None
        // elif corresponding.Length = 4 then
        //     solve corresponding.[0] corresponding.[1] corresponding.[2] corresponding.[3] 
        else    
            estimate corresponding

    let algebraicError (lr : (V2d * V2d)[]) (h : M33d) =
        if lr.Length = 0 then
            0.0
        else
            let H = h/h.M22
            lr |> Array.averageBy (fun (l,r) ->
                let p = H*V3d(l,1.0)
                (r - p.XY/p.Z).LengthSquared
            )   |> sqrt

    // Malis, Ezio, and Manuel Vargas. Deeper understanding of the homography decomposition for vision-based control. Diss. INRIA, 2007.
    let decompose (H : M33d) (lIntern : Projection) (rIntern : Projection) (references : list<V2d * V2d>) =
        let K1 = Projection.toTrafo rIntern
        let K0 = Projection.toTrafo lIntern
        let hNorm = K1.Backward * H * K0.Forward
        
        let eps = 1e-8

        //Log.warn "%A" hNorm.Det

        if hNorm.IsIdentity eps then
            [ CameraMotion.zero ]
            
        else

            let m = hNorm.Transposed * hNorm
            let a0 = m.M01*m.M01*m.M22 + m.M02*m.M02*m.M11 + m.M12*m.M12*m.M00 - m.M00*m.M11*m.M22 - 2.0*m.M01*m.M02*m.M12
            let a1 = m.M00*m.M11 + m.M00*m.M22 + m.M11*m.M22 - m.M01*m.M01 - m.M02*m.M02 - m.M12*m.M12
            let a2 = -(m.M00 + m.M11 + m.M22)
            let struct (l0, l1, l2) = Polynomial.RealRootsOfNormed(a2, a1, a0)

            let arr = Array.sort [| l0; l1; l2 |]
            let gamma = sqrt arr.[1]

            let H = hNorm / gamma
            
            // negative determinant of minor
            let inline minorOpp (M : M33d) x y =
                let inline m i j = M.[i-1, j-1]

                let x1 = if x = 1 then 2 else 1
                let y1 = if y = 1 then 2 else 1

                let x2 = if x = 3 then 2 else 3
                let y2 = if y = 3 then 2 else 3

                let v = - (m x1 y1 * m x2 y2 - m x1 y2 * m x2 y1)
                if abs v < eps then 0.0 else v  //prevent possible sign flip around 0.0
            
            let S = H.Transposed * H - M33d.Identity

            if abs S.NormMax < eps then
                
                let motion =
                    let rot = Rot3d.FromM33d(H.ToOrthoNormal(),eps)
                    let trn = V3d.Zero
                    Euclidean3d(rot,trn)

                [ { trafo = motion; isNormalized = false } ]
            else

                let inline s i j = S.[i-1,j-1]

                let M11 = minorOpp S 1 1
                let M22 = minorOpp S 2 2
                let M33 = minorOpp S 3 3
                let M12 = minorOpp S 1 2
                let M13 = minorOpp S 1 3
                let M23 = minorOpp S 2 3
        
                // find non-zero component
                let abs11 = abs (s 1 1)
                let abs22 = abs (s 2 2)
                let abs33 = abs (s 3 3)
                let maxIdx = 
                    if abs11<abs22 then
                        if abs22<abs33 then
                            3
                        else
                            2
                    else
                        if abs11<abs33 then
                            3
                        else
                            1

                // find normal vectors from non-zero components
                let sqrt11 = sqrt M11
                let sqrt22 = sqrt M22
                let sqrt33 = sqrt M33

                let eps12 = sign1 M12
                let eps13 = sign1 M13
                let eps23 = sign1 M23

                let npa, npb =
                    if maxIdx = 1 then
                        V3d(
                            s 1 1,
                            s 1 2 + sqrt33,
                            s 1 3 + eps23 * sqrt22
                        ),
                        V3d(
                            s 1 1,
                            s 1 2 - sqrt33,
                            s 1 3 - eps23 * sqrt22
                        )
                    elif maxIdx = 2 then
                        V3d(
                            s 1 2 + sqrt33,
                            s 2 2,
                            s 2 3 - eps13 * sqrt11
                        ),
                        V3d(
                            s 1 2 - sqrt33,
                            s 2 2,
                            s 2 3 + eps13 * sqrt11
                        )
                    elif maxIdx = 3 then
                        V3d(
                            s 1 3 + eps12 * sqrt22,
                            s 2 3 + sqrt11,
                            s 3 3
                        ),
                        V3d(
                            s 1 3 - eps12 * sqrt22,
                            s 2 3 - sqrt11,
                            s 3 3
                        )
                    else failwith ""
        
                let na = npa.Normalized
                let nb = npb.Normalized

                // find the translation* vectors from the normal vectors
                let trace = s 1 1 + s 2 2 + s 3 3
                let v = 2.0 * sqrt (1.0 + trace - M11 - M22 - M33) 

                let maxii_r = sign1 (s maxIdx maxIdx) * sqrt (2.0 + trace + v)
                let n_t = sqrt (2.0 + trace - v)

                let tastar = 0.5 * n_t * (maxii_r * nb - n_t * na)
                let tbstar = 0.5 * n_t * (maxii_r * na - n_t * nb)

                // use the translation* vectors and normals to get rotation matrix and real translation vectors
                let inline computeRotation tstar n =
                    H * (M33d.Identity - (2.0 / v) * Vec.outerProduct tstar n)
            
                let Ra = computeRotation tastar na
                let Rb = computeRotation tbstar nb

                let ta = Ra * tastar
                let tb = Rb * tbstar

                let motionOf (R : M33d, t : V3d, n : V3d) =
                    try
                        //https://github.com/JimZhou-001/opencv/commit/dc39ab67fc489ccf5dafbeb6ecbff211f9236b7f
                        let Mo = if R.Det <= 0.0 then -1.0 * R else R

                        let M = Mo.ToOrthoNormal()
                        let trafo = Euclidean3d(Rot3d.FromM33d(M,eps),t.Normalized)

                        let isValid =
                            references |> List.forall (fun (l,r) -> 
                                let vl = V3d(l,-1.0)
                                let rl = Ray3d(V3d.OOO, vl |> Vec.normalize)

                                let oc = trafo.InvTransformPos V3d.Zero
                                let vr = trafo.InvTransformPos (V3d(r,-1.0))
                                let rr = Ray3d(oc, (vr - oc) |> Vec.normalize)
                        
                                let m = rl.GetMiddlePoint rr

                                let t1 = rl.GetTOfProjectedPoint m
                                let t2 = rr.GetTOfProjectedPoint m

                                let res = t1 > 0.0 && t2 > 0.0
                                res
                            )
                        if isValid then
                            Some { trafo = trafo; isNormalized = true }
                        else
                            None
                    with _ ->
                        None
                let result = 
                    List.choose motionOf [
                        (Ra,ta,na)
                        (Ra,-ta,-na)
                        (Rb,tb,nb)
                        (Rb,-tb,-nb)
                    ]
                match result with
                    | [] -> 
                        //Log.error "all is wrong"
                        //System.Diagnostics.Debugger.Launch() |> ignore
                        []
                    | r -> r
    
    module SacHelpers =
        
        let checkPolygons (poly1 : V2d[]) (poly2 : V2d[]) =
            let p1 = Polygon2d(poly1).ComputeConvexHullIndexPolygon()
            let p2 = Polygon2d(poly2).ComputeConvexHullIndexPolygon()
            if p1.PointCount < 4 || p2.PointCount < 4 then
                false
            else
                let i1 = p1.GetIndexArray()
                let i2 = p2.GetIndexArray()
                let p1Good = 
                    let a0 = Triangle2d(poly1.[i1.[0]], poly1.[i1.[1]], poly1.[i1.[2]]).Area
                    let a1 = Triangle2d(poly1.[i1.[0]], poly1.[i1.[2]], poly1.[i1.[3]]).Area
                    let f = max a0 a1 / min a0 a1
                    f < 5.0
                    
                let p2Good = 
                    let a0 = Triangle2d(poly2.[i2.[0]], poly2.[i2.[1]], poly2.[i2.[2]]).Area
                    let a1 = Triangle2d(poly2.[i2.[0]], poly2.[i2.[2]], poly2.[i2.[3]]).Area
                    let f = max a0 a1 / min a0 a1
                    f < 5.0

                p1Good && p2Good

        let construct (ms : (V2d * V2d)[]) =
            recover ms |> Option.map ( fun m -> Trafo2d(m, m.Inverse)) |> Option.toList

        let solve (features : array<ImagePoint * ImagePoint>) =
            let ps = features |> Array.map (fun (l,r) -> l.ndc, r.ndc)
            let ls, rs = Array.unzip ps

            if checkPolygons ls rs then
                construct ps
            else
                []

        let isInlierUnique (eps : float) (h : Trafo2d) (l : ImagePoint) (r : ImagePoint) =
            let p0 = h.Forward.TransformPosProj l.ndc
            if V2d.Distance(r.ndc, p0) < eps then
                let q0 = h.Backward.TransformPosProj r.ndc
                if V2d.Distance(l.ndc, q0) < eps then
                    true
                else
                    false
            else
                false

        let isInlier maxNdc (sol : Trafo2d)  (l : ImagePoint, r : array<ImagePoint>) =
            if r.Length = 1 then
                isInlierUnique maxNdc sol l r.[0]

            elif r.Length = 2 then 
                if isInlierUnique maxNdc sol l r.[0] then
                    if isInlierUnique maxNdc sol l r.[1] then false
                    else true
                else
                    isInlierUnique maxNdc sol l r.[1]
            else
                let mutable found = false
                let mutable i = 0
                while i < r.Length do
                    if isInlierUnique maxNdc sol l r.[i] then
                        if found then
                            found <- false
                            i <- r.Length
                        else
                            found <- true
                    i <- i + 1
                found
 
        let countInliers maxNdc (sol : Trafo2d) (test : array<ImagePoint * ImagePoint[]>) =
            let mutable cnt = 0
            for tup in test do
                if isInlier maxNdc sol tup then
                    cnt <- cnt + 1
            cnt

        let getInliers maxNdc (sol : Trafo2d) (test : array<ImagePoint * ImagePoint[]>) =
            let result = System.Collections.Generic.List<int>()

            for i in 0 .. test.Length - 1 do
                if isInlier maxNdc sol test.[i] then
                    result.Add i
            
            CSharpList.toArray result

        let getSolution maxNdc (lProj : Projection) (rProj : Projection) (res : RansacResult<ImagePoint * ImagePoint,ImagePoint * ImagePoint[],Trafo2d>) =
            let H = res.value.Forward
            let matches =
                res.inliers |> Array.map (fun i ->
                    let (l,rs) = res.test.[i]
                    let r = rs |> Array.find (fun r -> isInlierUnique maxNdc res.value l r)
                    (l,r)
                )

            let referencePoints = res.model |> Seq.map (fun (l,r) -> l.ndc, r.ndc) |> Seq.toList
            let decomp = decompose H lProj rProj referencePoints

            decomp |> List.map ( fun motion -> 
                {
                    motion = motion |> CameraMotion.normalize
                    tag = "H"
                    matches = matches
                }
            )

        //let isInlierUnique (cfg : FeatureMatchConfig) (h : Trafo2d) (l : Feature) (r : Feature) =
        //    let p0 = h.Forward.TransformPosProj l.coord
        //    if V2d.Distance(r.coord, p0) < cfg.maxNdcDistance then
        //        let q0 = h.Backward.TransformPosProj r.coord
        //        if V2d.Distance(l.coord, q0) < cfg.maxNdcDistance then
        //            let p1 = h.Forward.TransformPosProj (l.coord + l.size * V2d(cos l.angle, sin l.angle))
        //            let rDir = p1 - p0
        //            let rSize = Vec.length rDir

        //            let f = max rSize r.size / min rSize r.size
        //            if f <= cfg.maxScaleFactor then
        //                let angle = Vec.dot (V2d(cos r.angle, sin r.angle)) (rDir / rSize) |> acos
        //                abs angle < cfg.maxAngle
        //            else
        //                false
        //        else
        //            false
        //    else
        //        false

        //let isInlier (cfg : FeatureMatchConfig) (sol : Trafo2d)  (l : Feature, r : array<Feature>) =
        //    if r.Length = 1 then
        //        isInlierUnique cfg sol l r.[0]

        //    elif r.Length = 2 then 
        //        if isInlierUnique cfg sol l r.[0] then
        //            if isInlierUnique cfg sol l r.[1] then false
        //            else true
        //        else
        //            isInlierUnique cfg sol l r.[1]
        //    else
        //        let mutable found = false
        //        let mutable i = 0
        //        while i < r.Length do
        //            if isInlierUnique cfg sol l r.[i] then
        //                if found then
        //                    found <- false
        //                    i <- r.Length
        //                else
        //                    found <- true
        //            i <- i + 1
        //        found
 
        //let getSolution (cfg : FeatureMatchConfig) (res : RansacResult<Feature * Feature,Feature * Feature[],Trafo2d>) =
        //    let H = res.value.Forward
        //    let matches =
        //        res.inliers |> Array.map (fun i ->
        //            let (l,rs) = res.test.[i]
        //            let r = rs |> Array.find (fun r -> isInlierUnique cfg res.value l r)
        //            (l,r)
        //        )

        //    let referencePoints = res.model |> Seq.map (fun (l,r) -> l.coord, r.coord) |> Seq.toList
        //    let motions = decompose H cfg.projection cfg.projection referencePoints

        //    let matches = 
        //        motions |> List.map (fun motion ->
        //            {
        //                motion = motion
        //                tag = "H"
        //                matches = matches
        //            }
        //        )

        //    matches
          
    module Ransac =
        open SacHelpers

        let problem (lProj : Projection) (rProj : Projection) (maxNdcDist : float)  =
            let countInliers (sol : Trafo2d) (test : array<ImagePoint * ImagePoint[]>) =
                let mutable cnt = 0
                for tup in test do
                    if isInlier maxNdcDist sol tup then
                        cnt <- cnt + 1
                cnt

            let getInliers (sol : Trafo2d) (test : array<ImagePoint * ImagePoint[]>) =
                let result = System.Collections.Generic.List<int>()

                for i in 0 .. test.Length - 1 do
                    if isInlier maxNdcDist sol test.[i] then
                        result.Add i
                
                CSharpList.toArray result

            {
                RansacProblem.neededSamples   = 4
                RansacProblem.solve           = solve
                RansacProblem.countInliers    = countInliers
                RansacProblem.getInliers      = getInliers
                RansacProblem.getSolution     = getSolution maxNdcDist lProj rProj
            } :> IRansacProblem<_,_,_>
            
    module Mulsac =
        open SacHelpers

        let same (eps : float) (l : Trafo2d) (r : Trafo2d) (lm : (ImagePoint*ImagePoint)[]) (rm : (ImagePoint*ImagePoint)[]) =
            lm |> Array.forall (fun (lf,rf) -> isInlierUnique eps r lf rf) &&
            rm |> Array.forall (fun (lf,rf) -> isInlierUnique eps l lf rf) 
                
        let group (t : float) (result : seq<RansacResult<(ImagePoint * ImagePoint),(ImagePoint * ImagePoint[]),list<CameraMatch>>>) =
            let cand =
                result |> Seq.collect (fun res ->
                    res.value |> Seq.map ( fun v -> 
                        { res with value = List.singleton v }
                    )
                ) |> Seq.toArray
                
            let similar (l : CameraMotion) (r : CameraMotion) = CameraMotion.similarity l r >= t
            let inline gm (g : RansacResult<(ImagePoint * ImagePoint),(ImagePoint * ImagePoint[]),list<CameraMatch>>) = (g.value |> List.head).motion
            let inline ms (g : RansacResult<(ImagePoint * ImagePoint),(ImagePoint * ImagePoint[]),list<CameraMatch>>) = (g.value |> List.head).matches

            let mutable groups = MapExt.empty
            for i in 0..cand.Length-1 do
                match groups |> MapExt.tryPick ( fun k v -> if similar (gm cand.[k]) (gm cand.[i]) then Some k else None ) with
                | None -> 
                    groups <- MapExt.add i [ cand.[i] ] groups
                | Some k -> 
                    groups <- MapExt.add k (cand.[i]::groups.[k]) groups
               
            let mergeTwo (l : RansacResult<(ImagePoint * ImagePoint),(ImagePoint * ImagePoint[]),list<CameraMatch>>) (r : RansacResult<(ImagePoint * ImagePoint),(ImagePoint * ImagePoint[]),list<CameraMatch>>) =
                {
                    data            = l.data
                    test            = l.test
                    value           = List.singleton {
                        motion = gm l
                        tag = "H"
                        matches = Array.append (ms l) (ms r) |> Array.distinct
                    }
                    modelIndices    = l.modelIndices
                    inliers         = Array.append l.inliers r.inliers |> Array.distinct
                    iterations      = l.iterations
                    time            = max l.time r.time
                }

            let merge (res : list<RansacResult<(ImagePoint * ImagePoint),(ImagePoint * ImagePoint[]),list<CameraMatch>>>) =
                match res |> Seq.tryHead with
                | None -> None
                | Some head -> res |> List.fold mergeTwo head |> Some

            let res = groups |> MapExt.values |> Seq.choose merge |> Seq.sortByDescending ( fun m -> m.inliers.Length )

            res
        
        let problem (lProj : Projection) (rProj : Projection) (maxNdc : float) (motionSimilarityEps : float) =
            
            let msProb = {
                neededSamples = 4
                solve = SacHelpers.solve
                countInliers = countInliers maxNdc
                getInliers = getInliers maxNdc
                getSolution = getSolution maxNdc lProj rProj
                same = same maxNdc
            }

            { new IRansacProblem<_,_,_> with
                member x.Solve(cfg,train,test) =
                    msProb.SolveMap(cfg,group motionSimilarityEps,train,Array.create train.Length 1.0,test)
            }