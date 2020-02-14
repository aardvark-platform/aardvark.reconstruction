namespace Aardvark.Reconstruction.Epipolar.Demo

open System
open Aardvark.Base
open Aardvark.Base.Incremental
open Aardvark.Base.Incremental.Operators
open Aardvark.Rendering.Text
open Aardvark.Base.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FShade
open Aardvark.Reconstruction

module Test = 
    let getBestFittingMot (c0 : Camera) (c1 : Camera) (mots : list<CameraMotion>) =
        mots |> List.maxBy (fun mot -> 
            let r = c0 + mot
            let dir = (1.0 + Vec.dot c1.Forward r.Forward)/2.0 * 1000.0
            let pos = -Vec.length (c1.Location - r.Location)
            (dir, pos)
        )

    let fundamentalChecker() =
        Aardvark.Init()
        Ag.initialize()
        
        let win = window {
            backend Backend.GL
            samples 8
        }

        let speed = Mod.init 1.0
        let focal = Mod.init 1.0
        let fov = focal |> Mod.map (fun focal -> (2.0 * atan( 1.0/focal )) * Constant.DegreesPerRadian)
        let aspect = win.Sizes |> Mod.map (fun s -> float s.X / float s.Y)
        let cv = CameraView.LookAt((V3d.IOO*6.0),V3d.OOO,V3d.OOI)
                    |> DefaultCameraController.controlWithSpeed speed win.Mouse win.Keyboard win.Time
        let frust = 
            Mod.custom (fun t -> 
                let aspect = aspect.GetValue(t)
                let fov = fov.GetValue(t)
                Frustum.perspective fov 0.1 100.0 aspect
            )
        let rview = 
            cv |> Mod.map (fun acv -> 
                CameraView.lookAt acv.Location (acv.Location + acv.Forward) V3d.OOI
            )
        let rproj = 
            Mod.custom (fun t ->
                let aspect = aspect.GetValue(t)
                let focal = focal.GetValue(t)
                let s = win.Sizes.GetValue(t)
                { 
                    focalLength = focal
                    aspect = aspect
                    distortion = {
                        principalPoint = V2d.Zero
                        imageSize = s
                        distortion = RadialDistortion2d.Identity
                    }
                }
            )
        let c1 = 
            Mod.map2 (fun v p ->
                { view = v; proj = p }
            ) rview rproj
        let c0 = Mod.init (c1.GetValue())

        let sameness = 
            Mod.map2 (fun real ref -> 
                Camera.sameness real ref
            ) c0 c1

        let F =
            Mod.map2 (fun c0 c1 -> 
                FundamentalMatrix.fromCameras c0 c1
            ) c0 c1

        let scale =
            Mod.map2 (fun (c0 : Camera) (c1 : Camera) -> 
                (c1.Location - c0.Location).Length
            ) c0 c1

        let mot =
            Mod.custom (fun t -> 
                let F = F.GetValue(t)
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let scale = scale.GetValue(t)
                match FundamentalMatrix.decompose F c0.proj c1.proj [] with
                | [] -> None
                | mots -> getBestFittingMot c0 c1 mots |> Some
            )
        let c1e = 
            Mod.custom (fun t -> 
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let mot = mot.GetValue(t)
                match mot with 
                | Some mot -> 
                    let scale = scale.GetValue(t)
                    { (c0 + mot*scale) with proj = c1.proj }
                | None -> 
                    c0
            )

        let riteText =
            Mod.custom (fun t -> 
                let focal = focal.GetValue(t)
                let fov = fov.GetValue(t)
                let frust = frust.GetValue(t)
                let s = win.Sizes.GetValue(t)
                let c1 = c1.GetValue(t)

                let aspect = frust |> Frustum.aspect
                String.concat "\n" [
                    sprintf "focal %.3f = fov %.1f°" focal fov
                    sprintf "aspect %.5f" aspect
                    sprintf "sizes [%d,%d]" s.X s.Y
                    sprintf "pos [%.2f,%.2f,%.2f]" c1.Location.X c1.Location.Y c1.Location.Z
                    sprintf "fwd [%.3f,%.3f,%.3f]" c1.Forward.X c1.Forward.Y c1.Forward.Z
                ]
            )
        let inline spaces (F : M33d) =
            let longest = 
                [F.M00; F.M01; F.M02; F.M10; F.M11; F.M12; F.M20; F.M21; F.M22]
                |> List.map int
                |> List.map string
                |> List.map String.length
                |> List.max
            fun (f : float) -> 
                let len = f |> int |> string |> String.length
                let num = longest - len + 1
                List.replicate num " " |> String.concat ""

        let inline printMat (F : M33d) =
            let sp = spaces F
            String.concat "\n" [
                sprintf " |%.6f%s%.6f%s%.6f|" F.M00 (sp F.M10) F.M10 (sp F.M20) F.M20
                sprintf " |%.6f%s%.6f%s%.6f|" F.M01 (sp F.M11) F.M11 (sp F.M21) F.M21
                sprintf " |%.6f%s%.6f%s%.6f|" F.M02 (sp F.M12) F.M12 (sp F.M22) F.M22
            ]

        let leftText =
            Mod.custom (fun t -> 
                let s = sameness.GetValue(t)
                let F = F.GetValue(t)
                let mot = mot.GetValue(t)
                let c1 = c1.GetValue(t)
                let c1e = c1e.GetValue(t)
                let correct = Camera.sameness c1 c1e
                let decompStr =
                    match mot with 
                    | None -> "No."
                    | Some mot -> 
                        let R : M33d = mot.trafo.Rot |> Rot3d.op_Explicit
                        let t = mot.trafo.Trans
                        String.concat "\n" [
                            sprintf "decomposed"
                            sprintf "R\n%s" (printMat R)
                            sprintf "t [%.3f,%.3f,%.3f]" t.X t.Y t.Z
                        ]

                String.concat "\n" [
                    sprintf "sameness %.5f" s
                    sprintf "F\n%s" (printMat F)
                    sprintf "%s" decompStr
                    sprintf "similarity %.5f" correct
                ]
            )

        win.Keyboard.DownWithRepeats.Values.Add (fun k ->
            match k with 
            | Keys.PageUp ->   transact(fun _ -> speed.Value <- speed.Value * 1.3)
            | Keys.PageDown -> transact(fun _ -> speed.Value <- speed.Value / 1.3)
            | Keys.OemPlus ->  transact(fun _ -> focal.Value <- focal.Value * 1.025)
            | Keys.OemMinus -> transact(fun _ -> focal.Value <- focal.Value / 1.025)
            | Keys.Enter -> transact(fun _ -> c0.Value <- c1.GetValue())
            | _ -> ()
        )

        let boxsg =
            Sg.box' C4b.Red (Box3d(V3d.NNN,V3d.III))    
            |> Sg.diffuseTexture DefaultTextures.checkerboard
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.diffuseTexture
                do! DefaultSurfaces.simpleLighting
            }

        let leftTextSg =
            Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White leftText
            |> Sg.scale 0.035
            |> Sg.translate -0.99 0.4 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let riteTextSg =
            Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White riteText
            |> Sg.scale 0.035
            |> Sg.translate 0.49 0.4 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let referenceCamSg =
            c0 |> Mod.map (fun cam -> 
                Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 cam C4b.Blue
            ) |> Sg.dynamic

        let sg =
            Sg.ofList [
                Sg.ofList [
                    boxsg
                    referenceCamSg
                ]   
                    //|> Sg.viewTrafo (cv |> Mod.map Aardvark.Base.CameraView.viewTrafo)
                    //|> Sg.projTrafo (frust |> Mod.map Frustum.projTrafo)
                    |> Sg.viewTrafo (c1e |> Mod.map (Camera.viewProjTrafo 0.1 100.0))
                    |> Sg.projTrafo (Mod.constant Trafo3d.Identity)
                leftTextSg
                riteTextSg
            ]

        win.Scene <- sg
        win.Run()

    let runManyAndRenderIfBad() =
        let tryRun i =
            
            let c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d = Generate.randomScene()
            let Ppoints2dc0 = Hpoints2dc0@Fpoints2dc0
            let Ppoints2dc1 = Hpoints2dc1@Fpoints2dc1
            let scale = (c1.Location - c0.Location).Length
            let fMatches = 
                List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Fpoints2dc0 Fpoints2dc1
            let hMatches =
                List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Hpoints2dc0 Hpoints2dc1
            let p3d = (Hpoints3d@Fpoints3d)
            let pMatches = 
                List.zip (Hpoints2dc1@Fpoints2dc1) p3d
                |> List.collect ( fun ((_,_,fs),(_,_,ps)) -> List.zip fs ps)
                |> List.toArray

            let hom = Homography.recover (hMatches |> List.toArray)
            let hmot =
                match hom with
                | None -> 
                    [CameraMotion.Zero]
                | Some h -> 
                    match Homography.decompose h c0.proj c1.proj [] with
                    | [] -> 
                        [CameraMotion.Zero]
                    | hs -> 
                        hs |> List.map (fun m -> m * scale)
            
            let fund = FundamentalMatrix.recover 1E-6 (fMatches |> List.toArray)
            let fmot,lsbr,rsbl =
                match fund with
                | None -> 
                    [CameraMotion.Zero],V2d.Zero,V2d.Zero
                | Some (F,lsbr,rsbl) -> 
                    match FundamentalMatrix.decompose F c0.proj c1.proj [] with
                    | [] -> 
                        [CameraMotion.Zero],lsbr,rsbl
                    | fs -> 
                        fs |> List.map (fun m -> m * scale),lsbr,rsbl

            let cp =
                match P6P.recover pMatches with
                | None -> 
                    c0
                | Some cp -> 
                    cp

            let ch = { (c0 + (getBestFittingMot c0 c1 hmot)) with proj = c1.proj }
            let cf = { (c0 + (getBestFittingMot c0 c1 fmot)) with proj = c1.proj }

            if 
             not ( Camera.approxEqual 1E-4 c1 cp ) ||
             not ( Camera.approxEqual 1E-4 c1 ch ) ||
             not ( Camera.approxEqual 1E-4 c1 cf ) then
                

                Log.error "ALL IS WRONG %d" i
                Log.error "cf %f" (Camera.sameness c1 cf)
                Log.error "ch %f" (Camera.sameness c1 ch)
                Log.error "cp %f" (Camera.sameness c1 cp)

                let ostr =
                    sprintf "ALL IS WRONG: %i\ncf %f ch %f cp %f\n" i (Camera.sameness c1 cf) (Camera.sameness c1 ch) (Camera.sameness c1 cp)
                    + sprintf "F-lsbr:%A\n" lsbr
                    + sprintf "F-rsbl:%A\n" rsbl

                Ag.initialize()
                Aardvark.Init()
                let win = window {
                    backend Backend.GL
                }
            
                Log.startTimed("Show")

                let vpo = Mod.map2 (fun (v : Trafo3d[]) (p : Trafo3d[]) -> v.[0] * p.[0]) win.View win.Proj
                let vp0 = c0 |> Mod.constant |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
                let vp1 = c1 |> Mod.constant |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
                let vph = ch |> Mod.constant |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
                let vpf = cf |> Mod.constant |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
                let vpp = cp |> Mod.constant |> Mod.map(Camera.viewProjTrafo 0.1 100.0)

                let current = Mod.init 0
                win.Keyboard.DownWithRepeats.Values.Add (fun k -> 
                    match k with
                    | Keys.Space -> 
                        transact (fun _ -> current.Value <- (current.Value + 1)%6)
                        match current.Value with 
                        | 0 -> Log.line "freefly"
                        | 1 -> Log.line "Camera 0 original"
                        | 2 -> Log.line "Camera 1 original"
                        | 3 -> Log.line "Camera 1 homography"
                        | 4 -> Log.line "Camera 1 fundamental"
                        | 5 -> Log.line "Camera 1 P6P"
                        | _ -> ()
                    | _ -> ()
                )

                let vt = current |> Mod.bind (fun c -> 
                    match c with
                    | 0 -> vpo
                    | 1 -> vp0
                    | 2 -> vp1
                    | 3 -> vph
                    | 4 -> vpf
                    | 5 -> vpp
                    | _ -> vpo
                )

                let statusText =
                    let t = current |> Mod.map (fun c -> 
                        match c with 
                        | 0 -> "Freefly"
                        | 1 -> "LeftCam"
                        | 2 -> "True RightCam"
                        | 3 -> "Homography"
                        | 4 -> "Fundamental"
                        | 5 -> "P6P"
                        | _ -> "?"
                    )
                    Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White t
                    |> Sg.scale 0.1
                    |> Sg.translate -0.9 -0.9 0.0
                    |> Sg.viewTrafo ~~Trafo3d.Identity
                    |> Sg.projTrafo ~~Trafo3d.Identity

                let printText =
                    Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White ~~ostr
                    |> Sg.scale 0.05
                    |> Sg.translate -0.9 0.9 0.0
                    |> Sg.viewTrafo ~~Trafo3d.Identity
                    |> Sg.projTrafo ~~Trafo3d.Identity

                let sg3d = 
                    (Generate.mkSg p3d)
                    |> Sg.viewTrafo vt
                    |> Sg.projTrafo (Mod.constant Trafo3d.Identity)

                let ftrSg = 
                    Mod.custom (fun t -> 
                        let current = current.GetValue(t)
                        let obs = 
                            match current with 
                            | 1 -> Ppoints2dc0
                            | 2 | 5 -> Ppoints2dc1 
                            | 4 -> Fpoints2dc1
                            | 3 -> Hpoints2dc1
                            | _ -> []
                        match obs with 
                        | [] -> Sg.empty 
                        | _ -> 
                            let obs = obs |> List.collect (fun (_,_,v) -> v)
                            let s1 = 
                                let ps = obs|> List.toArray |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                                let cs = Array.create ps.Length C4b.Black
                                IndexedGeometry(
                                    Mode = IndexedGeometryMode.PointList,
                                    IndexedAttributes = SymDict.ofList [
                                        DefaultSemantic.Positions, ps :> Array
                                        DefaultSemantic.Colors, cs :> Array
                                    ]
                                ) |> Sg.ofIndexedGeometry
                                  |> Sg.shader {
                                    do! DefaultSurfaces.vertexColor
                                    do! DefaultSurfaces.pointSprite
                                    do! DefaultSurfaces.pointSpriteFragment
                                  }
                                  |> Sg.uniform "PointSize" (Mod.constant 3.5)
                                  |> Sg.depthTest (Mod.constant DepthTestMode.None)
                            let s2 = 
                                let ps = obs |> List.toArray |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                                let cs = Array.create ps.Length C4b.White
                                IndexedGeometry(
                                    Mode = IndexedGeometryMode.PointList,
                                    IndexedAttributes = SymDict.ofList [
                                        DefaultSemantic.Positions, ps :> Array
                                        DefaultSemantic.Colors, cs :> Array
                                    ]
                                ) |> Sg.ofIndexedGeometry
                                  |> Sg.shader {
                                    do! DefaultSurfaces.vertexColor
                                    do! DefaultSurfaces.pointSprite
                                    do! DefaultSurfaces.pointSpriteFragment
                                  }
                                  |> Sg.uniform "PointSize" (Mod.constant 2.0)
                                  |> Sg.depthTest (Mod.constant DepthTestMode.None)
                                  |> Sg.pass (RenderPass.after "asd" RenderPassOrder.Arbitrary RenderPass.main)
                            Sg.ofList [s1;s2]
                    ) |> Sg.dynamic

                let frustSg =
                    Mod.custom (fun t -> 
                        let current = current.GetValue(t)
                        if current = 0 then
                            [
                                Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c0 C4b.Blue
                                Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c1 C4b.Blue
                                Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.50 ch C4b.Green
                                Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.51 cf C4b.Red
                                Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.52 cp (C4b(100,255,255,255))
                            ] |> Sg.ofList
                        else Sg.empty
                    ) |> Sg.dynamic

                let sg = 
                    Sg.ofList [
                        frustSg        
                        ftrSg
                        sg3d
                        statusText
                        printText
                    ]

                win.Scene <- sg
                Log.stop()
                win.Run()
                

        let num = 100000
        Log.startTimed "%d random tests" num
        for i in 0..num-1 do
            tryRun i
            if i%10 = 0 then
                Report.Progress(float i / float num)
        Report.Progress(1.0)
        Log.stop()

    let runManyExamples() =
        let tryAndAssert i =
            
            let c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d = Generate.randomScene()
            let scale = (c1.Location - c0.Location).Length

            let fMatches = 
                List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Fpoints2dc0 Fpoints2dc1
            let hMatches =
                List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Hpoints2dc0 Hpoints2dc1
            let p3d = (Hpoints3d@Fpoints3d)
            let pMatches = 
                List.zip (Hpoints2dc1@Fpoints2dc1) p3d
                |> List.collect ( fun ((_,_,fs),(_,_,ps)) -> List.zip fs ps)
                |> List.toArray

            let hom = Homography.recover (hMatches |> List.toArray)
            let hmot =
                match hom with
                | None -> 
                    [CameraMotion.Zero]
                | Some h -> 
                    match Homography.decompose h c0.proj c1.proj [] with
                    | [] -> 
                        [CameraMotion.Zero]
                    | hs -> 
                        hs |> List.map (fun m -> m * scale)
            
            let fund = FundamentalMatrix.recover 1E-6 (fMatches |> List.toArray)
            let fmot =
                match fund with
                | None -> 
                    [CameraMotion.Zero]
                | Some (F,lsbr,rsbl) -> 
                    match FundamentalMatrix.decompose F c0.proj c1.proj [] with
                    | [] -> 
                        [CameraMotion.Zero]
                    | fs -> 
                        fs |> List.map (fun m -> m * scale)

            let cp =
                match P6P.recover pMatches with
                | None -> 
                    c0
                | Some cp -> 
                    cp

            let ch = { (c0 + (getBestFittingMot c0 c1 hmot)) with proj = c1.proj }
            let cf = { (c0 + (getBestFittingMot c0 c1 fmot)) with proj = c1.proj }

            let pg = ( Camera.approxEqual 1E-4 c1 cp )
            let hg = ( Camera.approxEqual 1E-4 c1 ch )
            let fg = ( Camera.approxEqual 1E-4 c1 cf )

            if 
             not ( pg ) ||
             not ( hg ) ||
             not ( fg ) then
                let cfs =  (Camera.sameness c1 cf)
                let chs =  (Camera.sameness c1 ch)
                let cps =  (Camera.sameness c1 cp)
                Log.error "ALL IS WRONG %d" i
                Log.error "cf %f" cfs
                Log.error "ch %f" chs
                Log.error "cp %f" cps

                (c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d) |> Pickler.pickler.Pickle |> File.writeAllBytes @"C:\temp\dump.bin"

                failwithf "%A" (cfs+chs+cps)

        let num = 100000
        Log.startTimed "%d random tests" num
        for i in 0..num-1 do
            tryAndAssert i
            if i%10 = 0 then
                Report.Progress(float i / float num)
        Report.Progress(1.0)
        Log.stop()

    let singleRenderTest() =
        Ag.initialize()
        Aardvark.Init()
        let win = window {
            backend Backend.GL
        }

        let counter = Mod.init 0

        let stuff =
            counter |> Mod.map ( fun _ -> 
                Log.startTimed("Generate scene")
                //let c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d = Generate.randomScene()
                let (c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d) : Camera * Camera * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V3d>> * list<V3d * V3d * list<V3d>> = File.readAllBytes @"C:\temp\dump.bin" |> Pickler.pickler.UnPickle
                let Ppoints2dc0 = Hpoints2dc0@Fpoints2dc0
                let Ppoints2dc1 = Hpoints2dc1@Fpoints2dc1
                let scale = (c1.Location - c0.Location).Length
                Log.stop()

                Log.startTimed("Calculate epipolar")
                let fMatches = 
                    List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Fpoints2dc0 Fpoints2dc1
                let hMatches =
                    List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Hpoints2dc0 Hpoints2dc1
                let p3d = (Hpoints3d@Fpoints3d)
                let pMatches = 
                    List.zip (Hpoints2dc1@Fpoints2dc1) p3d
                    |> List.collect ( fun ((_,_,fs),(_,_,ps)) -> List.zip fs ps)
                    |> List.toArray


                let hom = Homography.recover (hMatches |> List.toArray)
                let hmot =
                    match hom with
                    | None -> 
                        Log.warn "No homography possible"
                        [CameraMotion.Zero]
                    | Some h -> 
                        match Homography.decompose h c0.proj c1.proj [] with //hMatches with
                        | [] -> 
                            Log.warn "Homography decompose failed"
                            [CameraMotion.Zero]
                        | hs -> 
                            Log.line "Decomposed Homography into %d motions" hs.Length
                            hs |> List.map (fun m -> m * scale)
                
                let fund = FundamentalMatrix.recover 1E-6 (fMatches |> List.toArray)
                let fmot =
                    match fund with
                    | None -> 
                        Log.warn "No fundamental possible"
                        [CameraMotion.Zero]
                    | Some (F,lsbr,rsbl) -> 
                        Log.line "lsbr %A" lsbr
                        Log.line "rsbl %A" rsbl
                        match FundamentalMatrix.decompose F c0.proj c1.proj [] with
                        | [] -> 
                            Log.warn "Fundamental decompose failed"
                            [CameraMotion.Zero]
                        | fs -> 
                            Log.line "Decomposed Fundamental into %d motions" fs.Length
                            fs |> List.map (fun m -> m * scale)

                let cp =
                    match P6P.recover pMatches with
                    | None -> 
                        Log.line "P6P failed"
                        c0
                    | Some cp -> 
                        Log.line "Recovered P6P camera"
                        cp

                let cf = { (c0 + (getBestFittingMot c0 c1 fmot)) with proj = c1.proj }
                let ch = { (c0 + (getBestFittingMot c0 c1 hmot)) with proj = c1.proj }

                let fu c = 
                    pMatches |> Array.sumBy (fun (o,p) -> 
                        (c |> Camera.unproject o).GetMinimalDistanceTo(p)
                    )

                if fu ch > 1E-6 then Log.error "fFFFFFFUUUUUUUUUUU ch"
                if fu cf > 1E-6 then Log.error "fFFFFFFUUUUUUUUUUU cf"
                if fu cp > 1E-6 then Log.error "fFFFFFFUUUUUUUUUUU cp"

                
                Log.line "cf %f" (Camera.sameness c1 cf)
                Log.line "ch %f" (Camera.sameness c1 ch)
                Log.line "cp %f" (Camera.sameness c1 cp)

                Log.stop()
                Generate.mkSg p3d,c0,c1,cp,ch,cf,Ppoints2dc0,Hpoints2dc1,Fpoints2dc1,Ppoints2dc1
            )
    
        let sg = Mod.map          (fun (x,_,_,_,_,_,_,_,_,_) -> x) stuff
        let c0 = Mod.map          (fun (_,x,_,_,_,_,_,_,_,_) -> x) stuff
        let c1 = Mod.map          (fun (_,_,x,_,_,_,_,_,_,_) -> x) stuff
        let cp = Mod.map          (fun (_,_,_,x,_,_,_,_,_,_) -> x) stuff
        let ch = Mod.map          (fun (_,_,_,_,x,_,_,_,_,_) -> x) stuff
        let cf = Mod.map          (fun (_,_,_,_,_,x,_,_,_,_) -> x) stuff
        let Ppoints2dc0 = Mod.map (fun (_,_,_,_,_,_,x,_,_,_) -> x) stuff
        let Hpoints2dc1 = Mod.map (fun (_,_,_,_,_,_,_,x,_,_) -> x) stuff
        let Fpoints2dc1 = Mod.map (fun (_,_,_,_,_,_,_,_,x,_) -> x) stuff
        let Ppoints2dc1 = Mod.map (fun (_,_,_,_,_,_,_,_,_,x) -> x) stuff
    
        Log.startTimed("Show")

        let vpo = Mod.map2 (fun (v : Trafo3d[]) (p : Trafo3d[]) -> v.[0] * p.[0]) win.View win.Proj
        let vp0 = c0 |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vp1 = c1 |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vph = ch |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vpf = cf |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vpp = cp |> Mod.map(Camera.viewProjTrafo 0.1 100.0)

        let current = Mod.init 0
        win.Keyboard.DownWithRepeats.Values.Add (fun k -> 
            match k with
            | Keys.Space -> 
                transact (fun _ -> current.Value <- (current.Value + 1)%6)
                match current.Value with 
                | 0 -> Log.line "freefly"
                | 1 -> Log.line "Camera 0 original"
                | 2 -> Log.line "Camera 1 original"
                | 3 -> Log.line "Camera 1 homography"
                | 4 -> Log.line "Camera 1 fundamental"
                | 5 -> Log.line "Camera 1 P6P"
                | _ -> ()
            | Keys.Enter -> 
                transact (fun _ -> counter.Value <- counter.Value + 1)
            | _ -> ()
        )

        let vt = current |> Mod.bind (fun c -> 
            match c with
            | 0 -> vpo
            | 1 -> vp0
            | 2 -> vp1
            | 3 -> vph
            | 4 -> vpf
            | 5 -> vpp
            | _ -> vpo
        )

        let statusText =
            let t = current |> Mod.map (fun c -> 
                match c with 
                | 0 -> "Freefly"
                | 1 -> "LeftCam"
                | 2 -> "True RightCam"
                | 3 -> "Homography"
                | 4 -> "Fundamental"
                | 5 -> "P6P"
                | _ -> "?"
            )
            Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White t
            |> Sg.scale 0.1
            |> Sg.translate -0.9 -0.9 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let sg3d = 
            sg
            |> Sg.dynamic
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (Mod.constant Trafo3d.Identity)

        let ftrSg = 
            Mod.custom (fun t -> 
                let current = current.GetValue(t)

                let Ppoints2dc0 = Ppoints2dc0.GetValue(t)
                let Hpoints2dc1 = Hpoints2dc1.GetValue(t)
                let Fpoints2dc1 = Fpoints2dc1.GetValue(t)
                let Ppoints2dc1 = Ppoints2dc1.GetValue(t)
                let obs = 
                    match current with 
                    | 1 -> Ppoints2dc0
                    | 2 | 5 -> Ppoints2dc1 
                    | 4 -> Fpoints2dc1
                    | 3 -> Hpoints2dc1
                    | _ -> []
                match obs with 
                | [] -> Sg.empty 
                | _ -> 
                    let obs = obs |> List.collect (fun (_,_,v) -> v)
                    let s1 = 
                        let ps = obs|> List.toArray |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                        let cs = Array.create ps.Length C4b.Black
                        IndexedGeometry(
                            Mode = IndexedGeometryMode.PointList,
                            IndexedAttributes = SymDict.ofList [
                                DefaultSemantic.Positions, ps :> Array
                                DefaultSemantic.Colors, cs :> Array
                            ]
                        ) |> Sg.ofIndexedGeometry
                          |> Sg.shader {
                            do! DefaultSurfaces.vertexColor
                            do! DefaultSurfaces.pointSprite
                            do! DefaultSurfaces.pointSpriteFragment
                          }
                          |> Sg.uniform "PointSize" (Mod.constant 3.5)
                          |> Sg.depthTest (Mod.constant DepthTestMode.None)
                    let s2 = 
                        let ps = obs |> List.toArray |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                        let cs = Array.create ps.Length C4b.White
                        IndexedGeometry(
                            Mode = IndexedGeometryMode.PointList,
                            IndexedAttributes = SymDict.ofList [
                                DefaultSemantic.Positions, ps :> Array
                                DefaultSemantic.Colors, cs :> Array
                            ]
                        ) |> Sg.ofIndexedGeometry
                          |> Sg.shader {
                            do! DefaultSurfaces.vertexColor
                            do! DefaultSurfaces.pointSprite
                            do! DefaultSurfaces.pointSpriteFragment
                          }
                          |> Sg.uniform "PointSize" (Mod.constant 2.0)
                          |> Sg.depthTest (Mod.constant DepthTestMode.None)
                          |> Sg.pass (RenderPass.after "asd" RenderPassOrder.Arbitrary RenderPass.main)
                    Sg.ofList [s1;s2]
            ) |> Sg.dynamic

        let frustSg =
            Mod.custom (fun t -> 
                let current = current.GetValue(t)
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let ch = ch.GetValue(t)
                let cf = cf.GetValue(t)
                let cp = cp.GetValue(t)
                if current = 0 then
                    [
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c0 C4b.Blue
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c1 C4b.Blue
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.50 ch C4b.Green
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.51 cf C4b.Red
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.52 cp (C4b(100,255,255,255))
                    ] |> Sg.ofList
                else Sg.empty
            ) |> Sg.dynamic

        let sg = 
            Sg.ofList [
                frustSg        
                ftrSg
                sg3d
                statusText
            ]

        win.Scene <- sg
        Log.stop()
        win.Run()
        
    [<EntryPoint>]
    let main argv =
        //fundamentalChecker()
        singleRenderTest()
        //runManyExamples()
        //runManyAndRenderIfBad()

        printfn "Hello World from F#!"
        0 // return an integer exit code