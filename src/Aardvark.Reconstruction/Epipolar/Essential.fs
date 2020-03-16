namespace Aardvark.Reconstruction

open Aardvark.Base

module EssentialMatrix =
    open MiniCV

    let private unproj (p : Projection) (pt : V2d) =
        Projection.unproject pt p |> Vec.xy

    let inline private toPlane (v : V3d) =
        let l = v.XY.Length
        Plane2d(v.XY / l, -v.Z / l)

    let recover (lIntern : Projection) (rIntern : Projection) (correspondences : array<V2d * V2d>) =
        let ls, rs = Array.unzip correspondences
        let ls = ls |> Array.map (unproj lIntern)
        let rs = rs |> Array.map (unproj rIntern)

        OpenCV.fivepoint ls rs 
            |> Array.toList
            |> List.choose (fun E -> 
                if abs E.Determinant < 1E-5 then
                    let Et = E.Transposed

                    let mutable rsbl = V2d.Zero
                    let a = E * V3d(1.0, 1.0, 1.0)   |> toPlane
                    let b = E * V3d(-1.0, -1.0, 1.0) |> toPlane
                    a.Intersects(b, &rsbl) |> ignore
                        
                    let mutable lsbr = V2d.Zero
                    let a = Et * V3d(1.0, 1.0, 1.0)   |> toPlane
                    let b = Et * V3d(-1.0, -1.0, 1.0) |> toPlane
                    a.Intersects(b, &lsbr) |> ignore


                    Some(E, rsbl, lsbr)
                else
                    None
            )

    let decompose (E : M33d) (lIntern : Projection) (rIntern : Projection) (referencePoints : list<V2d * V2d>) =
        let referencePoints =
            referencePoints |> List.map (fun (l,r) ->
                (Projection.unproject l lIntern).XY, (Projection.unproject r rIntern).XY
            )

        FundamentalMatrix.decompose E Projection.identity Projection.identity referencePoints
        
    module Ransac =
        
        let private tangents (v : V2d) (c : Circle2d) (t0 : byref<V2d>) (t1 : byref<V2d>)=
            //http://jsfiddle.net/zxqCw/1/
            let d = c.Center - v
            let eps = 1e-6
            if d.Length - c.Radius |> abs < eps || d.Length < eps then
                false
            else
                let a = asin (c.Radius / d.Length)
                let b = atan2 d.Y d.X
                
                let t = b - a
                let ta = c.Center + V2d( c.Radius *  sin t, c.Radius * -cos t )
                
                let t = b + a
                let tb = c.Center + V2d( c.Radius * -sin t, c.Radius * cos t )
                
                t0 <- ta
                t1 <- tb

                true

        let private solve (lIntern : Projection) (rIntern : Projection) (data : array<ImagePoint * ImagePoint>) =
            let positions = data |> Array.map (fun (l,r) -> l.ndc, r.ndc)
            let ls, rs = Array.unzip positions 

            let lpoly = Polygon2d(ls).ComputeConvexHullIndexPolygon().ToPolygon2d()
            let rpoly = Polygon2d(rs).ComputeConvexHullIndexPolygon().ToPolygon2d()

            let inline toPlane (v : V3d) =
                let l = v.XY.Length
                Plane2d(v.XY / l, -v.Z / l)

            let lcirc =
                let P = lpoly.ComputePerimeter()
                let A = lpoly.ComputeArea()
                (2.0 * sqrt (Constant.Pi * A)) / P
            let rcirc =
                let P = rpoly.ComputePerimeter()
                let A = rpoly.ComputeArea()
                (2.0 * sqrt (Constant.Pi * A)) / P

            if lcirc > 0.4 && rcirc > 0.4 then
                recover lIntern rIntern positions
            else
                []

        let inline private locationPasst maxNdcDistance sol (l : ImagePoint) (r : ImagePoint) rsbl lsbr = 
            let pixel = Circle2d(l.ndc, maxNdcDistance * 0.5)
            let mutable ppu = Unchecked.defaultof<V2d>
            let mutable ppl = Unchecked.defaultof<V2d>
            if tangents rsbl pixel &ppu &ppl then
                let ppu = toPlane (sol * V3d(ppu, 1.0))
                let ppl = toPlane (sol * V3d(ppl, 1.0))
                    
                let ppu = 
                    if Vec.dot ppu.Normal ppl.Normal < 0.0 then     
                        ppu.Reversed
                    else
                        ppu                         
                    
                let h =
                    let u = ppu.Height(r.ndc)
                    let l = ppl.Height(r.ndc)
                    Range1d(min u l,max u l)
                h.Contains(0.0)
            else
                false
            
        let private isInlierUnique maxNdcDistance maxScaleFactor (sol : M33d, rsbl : V2d, lsbr : V2d) (l : ImagePoint) (r : ImagePoint) =
            if locationPasst maxNdcDistance sol l r rsbl lsbr && 
               locationPasst maxNdcDistance sol.Transposed r l lsbr rsbl then
                let lc = Circle2d(l.ndc, 0.00244) //5 / 2048 pixel
                let mutable p0 = Unchecked.defaultof<_>
                let mutable p1 = Unchecked.defaultof<_>
                if tangents rsbl lc &p0 &p1 then
                    let p0r = toPlane (sol * V3d(p0, 1.0))
                    let p1r = toPlane (sol * V3d(p1, 1.0))
                    
                    let h0 = p0r.Height r.ndc |> abs
                    let h1 = p1r.Height r.ndc |> abs
                                   
                    let c =
                        let mi = min h0 h1
                        let ma = max h0 h1
                        Range1d(mi / maxScaleFactor, ma * maxScaleFactor)
                    
                    let h = 0.00244
                    c.Contains(h)
                else
                    false 
            else 
                false

        let private isInlier maxNdcDistance maxScaleFactor sol (l : ImagePoint, r : array<ImagePoint>) =
            if r.Length = 1 then
                isInlierUnique maxNdcDistance maxScaleFactor sol l r.[0]

            elif r.Length = 2 then 
                if isInlierUnique maxNdcDistance maxScaleFactor sol l r.[0] then
                    if isInlierUnique maxNdcDistance maxScaleFactor sol l r.[1] then false
                    else true
                else
                    isInlierUnique maxNdcDistance maxScaleFactor sol l r.[1]
            else
                let mutable found = false
                let mutable i = 0
                while i < r.Length do
                    if isInlierUnique maxNdcDistance maxScaleFactor sol l r.[i] then
                        if found then
                            found <- false
                            i <- r.Length
                        else
                            found <- true
                    i <- i + 1
                found

                
        let private n = 720
        let private countInliers maxNdcDistance maxScaleFactor ((E : M33d, rsbl : V2d, lsbr : V2d) as sol) (test : array<ImagePoint * ImagePoint[]>) =
            let bins = Array.create n (struct (0, []))

            for ((l,rs) as tup) in test do
                let p2d = E * (V3d(l.ndc, 1.0))
                let b = ((atan2 p2d.Y p2d.X + Constant.Pi) / Constant.PiTimesTwo) * float n |> int
                let b = min b (n - 1)

                let struct(c,ls) = bins.[b]
                if c < 4 then
                    bins.[b] <- struct(c+1, tup :: ls)
                  
            let mutable cnt = 0
            for struct (c,checks) in bins do
                if c < 4 then
                    for tup in checks do
                       if isInlier maxNdcDistance maxScaleFactor sol tup then
                            cnt <- cnt + 1

            cnt

        let private getSolution maxNdcDistance maxScaleFactor (proj : Projection) (res : RansacResult<ImagePoint * ImagePoint, ImagePoint * ImagePoint[], M33d * V2d * V2d>) =
            let (E,_,_) = res.value
            let matches =
                res.inliers |> Array.map (fun i ->
                    let (l,rs) = res.test.[i]
                    let r = rs |> Array.find (fun r -> isInlierUnique maxNdcDistance maxScaleFactor res.value l r)
                    (l,r)
                )

            let referencePoints = res.model |> Seq.map (fun (l,r) -> l.ndc, r.ndc) |> Seq.toList
            let motions = decompose E proj proj referencePoints

            let matches = 
                motions |> List.map (fun motion ->
                    {
                        motion = motion
                        tag = "E"
                        matches = matches
                    }
                )

            matches

        let problem maxNdcDistance maxScaleFactor (proj : Projection) =
        
            let getInliers (sol : M33d * V2d * V2d) (test : array<ImagePoint * ImagePoint[]>) =
                let result = System.Collections.Generic.List<int>()

                for i in 0 .. test.Length - 1 do
                    if isInlier maxNdcDistance maxScaleFactor sol test.[i] then
                        result.Add i
                
                CSharpList.toArray result

            {
                neededSamples   = 5
                solve           = solve proj proj
                countInliers    = countInliers maxNdcDistance maxScaleFactor
                getInliers      = getInliers
                getSolution     = getSolution maxNdcDistance maxScaleFactor proj
            } :> IRansacProblem<_,_,_>
