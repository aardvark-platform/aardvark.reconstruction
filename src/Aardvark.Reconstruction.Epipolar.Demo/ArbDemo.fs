namespace Aardvark.Reconstruction.Epipolar.Demo

open System
open Aardvark.Base
open Aardvark.Base.Incremental
open Aardvark.Base.Incremental.Operators
open Aardvark.Rendering.Text
open Aardvark.Base.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Utilities
open Aardvark.Application.Slim
open FShade
open Aardvark.Reconstruction
open FsCheck

[<AutoOpen>]
module ArbDemo = 
    open Lala

    let sg3d (s : Scenario) =
        let pts3d = 
            s.pts3d
            |> Array.map (fun p -> 
                let s = Sphere3d(p,0.05)
                IndexedGeometryPrimitives.solidSubdivisionSphere s 2 C4b.Red
                |> Sg.ofIndexedGeometry
            ) 
            |> Sg.ofArray
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.vertexColor
                do! DefaultSurfaces.simpleLighting            
            }
        
        Sg.ofList [pts3d]

    let showScenario 
        (win : ISimpleRenderWindow)
        (scenario : IMod<Scenario>) 
        (cf : IMod<Option<Camera>>) 
        (ch : IMod<Option<Camera>>) 
        (cp : IMod<Option<Camera>>) =
        
        let c0 = scenario |> Mod.map (fun s -> s.cam0)
        let c1 = scenario |> Mod.map (fun s -> s.cam1)
        Log.startTimed("Show")

        let vpo = Mod.map2 (fun (v : Trafo3d[]) (p : Trafo3d[]) -> v.[0] * p.[0]) win.View win.Proj
        let vp0 = c0 |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vp1 = c1 |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vph = ch |> Mod.map(Option.map(Camera.viewProjTrafo 0.1 100.0))
        let vpf = cf |> Mod.map(Option.map(Camera.viewProjTrafo 0.1 100.0))
        let vpp = cp |> Mod.map(Option.map(Camera.viewProjTrafo 0.1 100.0))

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
            | 3 -> Mod.map2 (fun o t -> t |> Option.defaultValue o) vpo vph
            | 4 -> Mod.map2 (fun o t -> t |> Option.defaultValue o) vpo vpf
            | 5 -> Mod.map2 (fun o t -> t |> Option.defaultValue o) vpo vpp
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
            scenario
            |> Mod.map sg3d
            |> Sg.dynamic
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (Mod.constant Trafo3d.Identity)

        let ftrSg = 
            Mod.custom (fun t -> 
                let current = current.GetValue(t)
                let s = scenario.GetValue(t)

                let ls = s.matches |> Array.map fst
                let rs = s.matches |> Array.map snd
                let obs = 
                    match current with 
                    | 1 -> ls
                    | 2 | 3 | 4 | 5 -> rs
                    | _ -> [||]
                match obs with 
                | [||] -> Sg.empty 
                | _ -> 
                    let s1 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
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
                          |> Sg.uniform "PointSize" (Mod.constant 3.5)
                          |> Sg.depthTest (Mod.constant DepthTestMode.None)
                    let s2 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
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
                        yield Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c0 C4b.Blue
                        yield Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c1 C4b.Blue
                        yield! cf |> Option.map(fun c -> Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.50 c C4b.Green             )   |> Option.toList
                        yield! ch |> Option.map(fun c -> Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.51 c C4b.Red               )   |> Option.toList
                        yield! cp |> Option.map(fun c -> Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.52 c (C4b(100,255,255,255)))   |> Option.toList
                    ] |> Sg.ofList
                else Sg.empty
            ) |> Sg.dynamic

        let outlineSg =
            scenario |> Mod.map (fun s -> 
                match s.points with
                | AlmostLinearQuad(q,f) -> 
                    let q = flattenQuad q f
                    IndexedGeometryPrimitives.quad' q.P0 q.P1 q.P2 q.P3 C4b.Yellow    
                | AlmostFlatVolume(b,f) -> 
                    let b = flattenBox b f
                    IndexedGeometryPrimitives.Box.wireBox b C4b.Yellow
                | InQuad q -> 
                    IndexedGeometryPrimitives.quad' q.P0 q.P1 q.P2 q.P3 C4b.Red    
                | InVolume b -> 
                    IndexedGeometryPrimitives.Box.wireBox b C4b.Red            
            )
            |> Mod.map Sg.ofIndexedGeometry
            |> Sg.dynamic
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.thickLine
                do! DefaultSurfaces.vertexColor
            }
            |> Sg.uniform "LineWidth" ~~1.0
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (Mod.constant Trafo3d.Identity)

        let sg = 
            Sg.ofList [
                frustSg        
                ftrSg
                sg3d
                statusText
                outlineSg
            ]

        win.Scene <- sg
        Log.stop()
        win.Run()

    let singleArbTest() =

        Ag.initialize()
        Aardvark.Init()
        let win = window {
            backend Backend.GL
        }
        let rand = RandomSystem()

        let counter = Mod.init 0
        win.Keyboard.DownWithRepeats.Values.Add (function 
            | Keys.Enter -> 
                transact (fun _ -> counter.Value <- counter.Value + 1)
            | _ -> ()
        )
        
        let stuff =
            counter |> Mod.map ( fun _ -> 
                Log.startTimed("Generate Scenario")

                let scene = Gen.eval 0 (Random.StdGen(rand.UniformInt(),rand.UniformInt())) Lala.genScenario 
                //let scene : Scenario = @"D:\temp\scene.bin" |> File.readAllBytes |> Pickler.pickler.UnPickle

                Log.line "Scene:\n"
                //Log.line "C0:%A" scene.cam0
                Log.line "C1Trans:%A" scene.camtrans
                Log.line "C1Rot:%A" scene.camrot
                Log.line "pts3d:%A" scene.points
                let c0 = scene.cam0
                let c1 = scene.cam1
                let pMatches =
                    Array.zip scene.pts3d scene.matches
                    |> Array.map (fun (p3d,(l,r)) -> r,p3d)
                let scale = (c1.Location - c0.Location).Length
                Log.stop()

                Log.startTimed("Calculate epipolar")

                let matches = scene.matches
                let hom = Homography.recover matches
                let hmot =
                    match hom with
                    | None -> 
                        Log.warn "No homography possible"
                        []
                    | Some h -> 
                        match Homography.decompose h c0.proj c1.proj [] with
                        | [] -> 
                            Log.warn "Homography decompose failed"
                            []
                        | hs -> 
                            Log.line "Decomposed Homography into %d motions" hs.Length
                            hs |> List.map (fun m -> m * scale)
                
                let fmot =
                    let f = FundamentalMatrix.recover 1E-4 matches
                    match f with
                    | None -> 
                        Log.warn "No fundamental possible"
                        []
                    | Some (fund,lsbr,rsbl) -> 
                        Log.line "lsbr %A" lsbr
                        Log.line "rsbl %A" rsbl
                        match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                        | [] -> 
                            Log.warn "Fundamental decompose failed"
                            []
                        | fs -> 
                            Log.line "Decomposed Fundamental into %d motions" fs.Length
                            fs |> List.map (fun m -> m * scale)

                let cp : Option<Camera> =
                    match P6P.recover pMatches with
                    | None -> 
                        Log.warn "P6P failed"
                        None
                    | Some cp -> 
                        Log.line "Recovered P6P camera"
                        Some cp

                let cf = getBestFittingMot c0 c1 fmot |> Option.map (fun m -> { (c0 + m) with proj = c1.proj })
                let ch = getBestFittingMot c0 c1 hmot |> Option.map (fun m -> { (c0 + m) with proj = c1.proj })

                let fu c n = 
                    match c with
                    | None -> Log.warn "No camera: %s" n
                    | Some c -> 
                        let d = 
                            pMatches |> Array.sumBy (fun (o,p) -> 
                                (c |> Camera.unproject o).GetMinimalDistanceTo(p)
                            )
                        if d > 1E-0 then Log.error "bad cam: %s" n            
                        Log.line "sameness %s %f" n (Camera.sameness c1 c)

                fu ch "Homography"
                fu cf "Fundamental"
                fu cp "P6P"

                Log.stop()
                scene, cf, ch, cp
            )
    
        let scene = Mod.map       (fun (x,_,_,_) -> x) stuff
        let cf = Mod.map          (fun (_,x,_,_) -> x) stuff
        let ch = Mod.map          (fun (_,_,x,_) -> x) stuff
        let cp = Mod.map          (fun (_,_,_,x) -> x) stuff
        
        showScenario win scene cf ch cp


    let manyArbsAndRenderIfBad() =
        let mutable bad = None
        let tryRun() =
            let scene = Gen.eval 0 (Random.StdGen(rand.UniformInt(),rand.UniformInt())) Lala.genScenario 
            let c0 = scene.cam0
            let c1 = scene.cam1
            let pMatches =
                Array.zip scene.pts3d scene.matches
                |> Array.map (fun (p3d,(l,r)) -> r,p3d)
            let scale = (c1.Location - c0.Location).Length

            let matches = scene.matches
            let hom = Homography.recover matches
            let hmot =
                match hom with
                | None -> 
                    []
                | Some h -> 
                    match Homography.decompose h c0.proj c1.proj [] with
                    | [] -> 
                        []
                    | hs -> 
                        hs |> List.map (fun m -> m * scale)
            
            let fmot =
                let f = FundamentalMatrix.recover 1E-4 matches
                match f with
                | None -> 
                    []
                | Some (fund,lsbr,rsbl) -> 
                    match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                    | [] -> 
                        []
                    | fs -> 
                        fs |> List.map (fun m -> m * scale)

            let cp : Option<Camera> =
                match P6P.recover pMatches with
                | None -> 
                    None
                | Some cp -> 
                    Some cp

            let cf = getBestFittingMot c0 c1 fmot |> Option.map (fun m -> { (c0 + m) with proj = c1.proj })
            let ch = getBestFittingMot c0 c1 hmot |> Option.map (fun m -> { (c0 + m) with proj = c1.proj })

            let mutable testy = 0
            let fu c n = 
                match c with
                | None -> ()
                | Some c -> 
                    let d = 
                        pMatches |> Array.sumBy (fun (o,p) -> 
                            (c |> Camera.unproject o).GetMinimalDistanceTo(p)
                        )
                    if d > 1E-1 then testy <- testy+1

            fu ch "Homography"
            fu cf "Fundamental"
            //fu cp "P6P"

            if testy >= 2 then 
                Log.error "bad found:%A" scene
                //Pickler.pickler.Pickle(scene) |> File.writeAllBytes @"D:\temp\scene.bin"
                bad <- Some (scene, cf, ch, cp)

        let mutable ct = 0
        let max = 100000
        Log.startTimed "Running baddies"
        while bad |> Option.isNone && ct < max do
            Report.Progress(float ct/float max)
            tryRun()
            ct <- ct+1
        Report.Progress(1.0)
        Log.stop()        
        if bad |> Option.isNone then Log.line "juhu"
        else
            Ag.initialize()
            Aardvark.Init()
            let win = window {
                backend Backend.GL
            }

            Log.error "bad found:%A" ct
            let (scene, cf, ch, cp) = bad |> Option.get

            showScenario win ~~scene ~~cf ~~ch ~~cp